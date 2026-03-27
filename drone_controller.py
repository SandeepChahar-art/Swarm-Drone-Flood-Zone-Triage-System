#!/usr/bin/env python3
"""
drone_controller.py  ─  3D version
───────────────────────────────────
Adds full Z-axis (altitude) control.
Each drone flies to the formation goal's Z altitude (staggered by swarm_manager).
When assigned a task, it descends to RESCUE_ALT to simulate rescue hover.

Launch example:
  python drone_controller.py --ros-args -p drone_id:=0 -p start_x:=0.0 -p start_y:=0.0 -p start_z:=2.0
"""

import rclpy
from rclpy.node import Node
import json, math, random
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

RESCUE_ALT = 0.5   # metres — hover altitude when on a rescue task

# ── Wind model ────────────────────────────────────────────────────────────────
WIND_X     =  0.25   # m/s — steady wind eastward
WIND_Y     =  0.10   # m/s — steady wind northward
WIND_GUST  =  0.12   # m/s — max random gust amplitude

# ── Payload model ─────────────────────────────────────────────────────────────
EMPTY_MASS   = 1.0   # kg — drone alone
PAYLOAD_MASS = 0.8   # kg — rescue kit (life jacket, medicine, rope)

# Rescue kit contents
RESCUE_KIT = {
    "life_jacket":  {"weight": 0.28, "qty": 1, "desc": "Inflatable life jacket"},
    "medicine_box": {"weight": 0.18, "qty": 1, "desc": "Emergency meds (insulin, BP)"},
    "rope_25m":     {"weight": 0.22, "qty": 1, "desc": "25m rescue rope"},
    "food_water":   {"weight": 0.12, "qty": 1, "desc": "Emergency ration + water"},
}
KIT_TOTAL_MASS = sum(v['weight']*v['qty'] for v in RESCUE_KIT.values())
BASE_MAX_SPD = 1.5   # m/s — without payload
BASE_ATT     = 1.2   # attraction gain without payload


class DroneController(Node):

    STATE_PATROL = "patrol"
    STATE_TASKED = "tasked"
    STATE_IDLE   = "idle"

    def __init__(self):
        super().__init__('drone_controller')

        self.declare_parameter('drone_id',        0)
        self.declare_parameter('num_drones',      5)
        self.declare_parameter('max_speed',       1.5)
        self.declare_parameter('safe_distance',   1.5)
        self.declare_parameter('repulsion_gain',  2.0)
        self.declare_parameter('attraction_gain', 1.2)
        self.declare_parameter('damping',         0.85)
        self.declare_parameter('goal_tolerance',  0.3)
        self.declare_parameter('start_x',         0.0)
        self.declare_parameter('start_y',         0.0)
        self.declare_parameter('start_z',         2.0)
        self.declare_parameter('z_gain',          0.8)

        self.drone_id  = self.get_parameter('drone_id').value
        self.num_drones= self.get_parameter('num_drones').value
        self.max_speed = self.get_parameter('max_speed').value
        self.safe_dist = self.get_parameter('safe_distance').value
        self.rep_gain  = self.get_parameter('repulsion_gain').value
        self.att_gain  = self.get_parameter('attraction_gain').value
        self.damping   = self.get_parameter('damping').value
        self.goal_tol  = self.get_parameter('goal_tolerance').value
        self.z_gain    = self.get_parameter('z_gain').value

        ns = f'drone_{self.drone_id}'

        self.x  = self.get_parameter('start_x').value
        self.y  = self.get_parameter('start_y').value
        self.z  = self.get_parameter('start_z').value
        self.vx = self.vy = self.vz = 0.0

        self.state            = self.STATE_IDLE
        self.battery          = 100.0 - self.drone_id * 3.0
        self.task_id          = -1
        self.carrying_payload = False
        self.kit_remaining    = {k:dict(v) for k,v in RESCUE_KIT.items()}
        self.kit_mass_left    = KIT_TOTAL_MASS
        self.rescues_done     = 0
        self._gust_x          = 0.0
        self._gust_y          = 0.0
        self._gust_timer      = 0
        self.formation_goal   = None
        self.task_goal        = None
        self.neighbour_poses  = {}

        self.pose_pub   = self.create_publisher(PoseStamped, f'/{ns}/pose',    10)
        self.vel_pub    = self.create_publisher(Twist,        f'/{ns}/cmd_vel', 10)
        self.status_pub = self.create_publisher(String,       f'/{ns}/status',  10)

        self.create_subscription(PoseStamped, f'/{ns}/formation_goal',
                                 self._cb_formation_goal, 10)
        self.create_subscription(PoseStamped, f'/{ns}/task_goal',
                                 self._cb_task_goal, 10)

        for i in range(self.num_drones):
            if i != self.drone_id:
                self.create_subscription(
                    PoseStamped, f'/drone_{i}/pose',
                    lambda msg, did=i: self._cb_neighbour_pose(msg, did), 10)

        self.create_timer(0.05, self._control_loop)
        self.create_timer(10.0, self._drain_battery)

        self.get_logger().info(
            f'[drone_{self.drone_id}] 3D controller online '
            f'({self.x:.1f},{self.y:.1f},{self.z:.1f})')

    # ─── Callbacks ────────────────────────────────────────────────────────────

    def _cb_formation_goal(self, msg):
        self.formation_goal = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        if self.state == self.STATE_IDLE:
            self.state = self.STATE_PATROL

    def _cb_task_goal(self, msg):
        self.task_goal = (
            msg.pose.position.x,
            msg.pose.position.y,
            RESCUE_ALT,
        )
        try:
            self.task_id = int(msg.header.frame_id)
        except ValueError:
            self.task_id = -1
        self.carrying_payload = True   # picked up rescue kit
        self.state = self.STATE_TASKED
        self.get_logger().info(
            f'[drone_{self.drone_id}] Task {self.task_id} → '
            f'({self.task_goal[0]:.2f},{self.task_goal[1]:.2f},z={RESCUE_ALT})')

    def _cb_neighbour_pose(self, msg, drone_id):
        self.neighbour_poses[drone_id] = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    # ─── Control loop (20 Hz) ─────────────────────────────────────────────────

    def _control_loop(self):
        dt = 0.05

        if self.state == self.STATE_TASKED and self.task_goal is not None:
            goal = self.task_goal
        elif self.state == self.STATE_PATROL and self.formation_goal is not None:
            goal = self.formation_goal
        else:
            self._publish_pose()
            self._publish_status()
            return

        dx_g = goal[0] - self.x
        dy_g = goal[1] - self.y
        dz_g = goal[2] - self.z
        dist_xy = math.hypot(dx_g, dy_g)

        # Check arrival
        if dist_xy < self.goal_tol and abs(dz_g) < self.goal_tol:
            if self.state == self.STATE_TASKED:
                self.get_logger().info(
                    f'[drone_{self.drone_id}] Task {self.task_id} COMPLETE')
                self._publish_task_complete()
                self.task_goal        = None
                self.task_id          = -1
                self.carrying_payload = False
                self.rescues_done    += 1
                for item in ['life_jacket','medicine_box','rope_25m','food_water']:
                    if item in self.kit_remaining and self.kit_remaining[item]['qty']>0:
                        self.kit_remaining[item]['qty'] -= 1
                        self.kit_mass_left = sum(
                            v['weight']*v['qty'] for v in self.kit_remaining.values())
                        if self.kit_remaining[item]['qty']==0:
                            del self.kit_remaining[item]
                        break
                self.state            = self.STATE_PATROL
            fx_att = fy_att = fz_att = 0.0
        else:
            norm   = max(math.sqrt(dx_g**2 + dy_g**2 + dz_g**2), 0.01)
            fx_att = self.att_gain * (dx_g / norm)
            fy_att = self.att_gain * (dy_g / norm)
            fz_att = self.z_gain   * (dz_g / abs(dz_g) if abs(dz_g) > 0.05 else dz_g)

        # XY repulsion
        fx_rep = fy_rep = 0.0
        for nid, (nx, ny, nz) in self.neighbour_poses.items():
            dx_n = self.x - nx
            dy_n = self.y - ny
            d_xy = math.hypot(dx_n, dy_n)
            if d_xy < 0.01:
                continue
            if d_xy < self.safe_dist:
                mag    = self.rep_gain * ((self.safe_dist - d_xy) / (d_xy**2 + 0.1))
                fx_rep += mag * (dx_n / d_xy)
                fy_rep += mag * (dy_n / d_xy)

        # ── Wind disturbance ─────────────────────────────────────────────────
        self._gust_timer += 1
        if self._gust_timer >= 40:   # new gust every ~2 s at 20 Hz
            self._gust_x = random.uniform(-WIND_GUST, WIND_GUST)
            self._gust_y = random.uniform(-WIND_GUST, WIND_GUST)
            self._gust_timer = 0
        fx_wind = WIND_X + self._gust_x
        fy_wind = WIND_Y + self._gust_y

        # ── Payload: reduces speed and agility when carrying rescue kit ────────
        if self.carrying_payload:
            eff_mass   = EMPTY_MASS + self.kit_mass_left
            eff_spd    = BASE_MAX_SPD / max(eff_mass,0.5)
            eff_att    = BASE_ATT     / max(eff_mass,0.5)
        else:
            eff_mass   = EMPTY_MASS
            eff_spd    = BASE_MAX_SPD
            eff_att    = BASE_ATT

        # Scale attraction force by effective gain
        fx_att = fx_att * (eff_att / BASE_ATT)
        fy_att = fy_att * (eff_att / BASE_ATT)

        # Velocity integration
        self.vx = self.damping * self.vx + (fx_att + fx_rep + fx_wind) * dt
        self.vy = self.damping * self.vy + (fy_att + fy_rep + fy_wind) * dt
        self.vz = self.damping * self.vz + fz_att * dt

        spd_xy = math.hypot(self.vx, self.vy)
        if spd_xy > eff_spd:
            self.vx = self.vx / spd_xy * eff_spd
            self.vy = self.vy / spd_xy * eff_spd
        self.vz = max(-0.8, min(0.8, self.vz))

        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z  = max(0.1, self.z + self.vz * dt)

        self._publish_cmd_vel()
        self._publish_pose()
        self._publish_status()

    # ─── Publishers ───────────────────────────────────────────────────────────

    def _publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        self.pose_pub.publish(msg)

    def _publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.linear.z = self.vz
        self.vel_pub.publish(msg)

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            "battery": round(self.battery, 1),
            "state":   self.state,
            "task_id": self.task_id,
            "z":       round(self.z, 2),
            "payload":        self.carrying_payload,
            "kit_mass_left":  round(self.kit_mass_left,3),
            "kit_items_left": list(self.kit_remaining.keys()),
            "rescues_done":   self.rescues_done,
            "wind_x":  round(WIND_X + self._gust_x, 2),
            "wind_y":  round(WIND_Y + self._gust_y, 2),
        })
        self.status_pub.publish(msg)

    def _publish_task_complete(self):
        msg = String()
        msg.data = json.dumps({
            "battery": round(self.battery, 1),
            "state":   "task_complete",
            "task_id": self.task_id,
            "z":       round(self.z, 2),
        })
        self.status_pub.publish(msg)

    def _drain_battery(self):
        drain = 1.0
        # Fighting wind costs extra power
        wind_load = math.hypot(WIND_X, WIND_Y) * 0.6
        drain += wind_load
        # Carrying payload costs extra power
        if self.carrying_payload:
            drain += 0.8
        self.battery = max(0.0, self.battery - drain)
        if self.battery < 20.0:
            self.get_logger().warn(
                f'[drone_{self.drone_id}] LOW BATTERY: {self.battery:.0f}%')


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
