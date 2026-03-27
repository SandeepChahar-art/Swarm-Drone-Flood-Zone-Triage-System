#!/usr/bin/env python3
"""
swarm_manager.py  ─  3D version
────────────────────────────────
Formation offsets now include a Z (altitude) component so each drone
flies at a unique altitude layer, preventing mid-air collisions.

Altitude layers  (BASE_ALT + oz):
  Drone 0  →  2.0 m  (lowest / lead)
  Drone 1  →  2.5 m
  Drone 2  →  2.5 m
  Drone 3  →  3.0 m
  Drone 4  →  3.0 m  (highest / rear)

Auction formula (unchanged):
  score = 0.5 × urgency + 0.4 × (1/distance) + 0.1 × (battery/100)
"""

import rclpy
from rclpy.node import Node
import json, math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

BASE_ALT = 2.0   # base cruise altitude (metres)

# Wind — must match drone_controller.py
WIND_X = 0.25
WIND_Y = 0.10


class SwarmManager(Node):

    def __init__(self):
        super().__init__('swarm_manager')

        self.declare_parameter('num_drones',     5)
        self.declare_parameter('formation',      'triangle')
        self.declare_parameter('max_speed',      1.5)
        self.declare_parameter('goal_tolerance', 0.5)

        self.num_drones = self.get_parameter('num_drones').value
        self.formation  = self.get_parameter('formation').value
        self.max_speed  = self.get_parameter('max_speed').value
        self.wp_tol     = self.get_parameter('goal_tolerance').value

        # (ox, oy, oz) — oz is altitude offset from BASE_ALT
        self.FORMATIONS = {
            'triangle': [
                ( 0.0,  2.0, 0.0),   # drone 0 — front tip
                (-2.0, -1.0, 0.5),   # drone 1 — left rear
                ( 2.0, -1.0, 0.5),   # drone 2 — right rear
                (-1.0,  0.5, 1.0),   # drone 3 — inner left
                ( 1.0,  0.5, 1.0),   # drone 4 — inner right
            ],
            'square': [
                (-2.0,  2.0, 0.0),
                ( 2.0,  2.0, 0.5),
                (-2.0, -2.0, 0.5),
                ( 2.0, -2.0, 1.0),
                ( 0.0,  0.0, 1.0),
            ],
            'v_shape': [
                ( 0.0,  0.0, 0.0),
                (-1.5,  1.5, 0.5),
                ( 1.5,  1.5, 0.5),
                (-3.0,  3.0, 1.0),
                ( 3.0,  3.0, 1.0),
            ],
            'diamond': [
                ( 0.0,  2.5, 0.0),
                (-2.5,  0.0, 0.5),
                ( 2.5,  0.0, 0.5),
                ( 0.0, -2.5, 1.0),
                ( 0.0,  0.0, 1.0),
            ],
            'line': [
                (-4.0, 0.0, 0.0),
                (-2.0, 0.0, 0.3),
                ( 0.0, 0.0, 0.6),
                ( 2.0, 0.0, 0.9),
                ( 4.0, 0.0, 1.2),
            ],
            # 3-D exclusive formation — drones at very different altitudes
            'stack': [
                ( 0.0,  0.0, 0.0),
                ( 0.5,  0.0, 0.8),
                (-0.5,  0.0, 1.6),
                ( 0.0,  0.5, 2.4),
                ( 0.0, -0.5, 3.2),
            ],
        }

        # Patrol waypoints (x, y) — same as 2-D
        self.waypoints = [
            (0.0, 0.0), (8.0, 0.0), (8.0, 4.0), (0.0, 4.0),
            (0.0, 8.0), (8.0, 8.0), (8.0, 12.0),(0.0, 12.0),
        ]
        self.wp_idx   = 0
        self.centre_x = 0.0
        self.centre_y = 0.0

        self.poses    = {i: (0.0, 0.0, BASE_ALT) for i in range(self.num_drones)}
        self.statuses = {i: {"battery": 100.0, "state": "idle", "task_id": -1, "z": BASE_ALT}
                         for i in range(self.num_drones)}

        self.pending_tasks = {}
        self.active_tasks  = {}

        # Publishers
        self.formation_goal_pubs = {
            i: self.create_publisher(PoseStamped, f'/drone_{i}/formation_goal', 10)
            for i in range(self.num_drones)
        }
        self.task_goal_pubs = {
            i: self.create_publisher(PoseStamped, f'/drone_{i}/task_goal', 10)
            for i in range(self.num_drones)
        }
        self.centre_pub = self.create_publisher(PoseStamped, '/swarm/centre', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/swarm/detected_task', self._cb_task,      10)
        self.create_subscription(String,      '/swarm/change_formation', self._cb_formation, 10)

        for i in range(self.num_drones):
            self.create_subscription(
                PoseStamped, f'/drone_{i}/pose',
                lambda msg, did=i: self._cb_pose(msg, did), 10)
            self.create_subscription(
                String, f'/drone_{i}/status',
                lambda msg, did=i: self._cb_status(msg, did), 10)

        self.create_timer(0.1, self._manager_loop)
        self.get_logger().info(
            f'[swarm_manager] 3D Online — {self.num_drones} drones, '
            f'formation={self.formation}, base_alt={BASE_ALT}m')

    # ─── Callbacks ────────────────────────────────────────────────────────────

    def _cb_pose(self, msg, did):
        self.poses[did] = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    def _cb_status(self, msg, did):
        try:
            data = json.loads(msg.data)
            self.statuses[did] = data
            if data.get('state') == 'task_complete':
                raw = str(data.get('task_id', -1))
                key = None
                if raw in self.active_tasks:
                    key = raw
                elif raw.lstrip('-').isdigit() and int(raw) in self.active_tasks:
                    key = int(raw)
                if key is not None:
                    del self.active_tasks[key]
                    self.get_logger().info(
                        f'[swarm_manager] Task {raw} complete by drone {did}')
                    self._try_assign_pending()
        except json.JSONDecodeError:
            pass

    def _cb_task(self, msg):
        try:
            parts   = msg.header.frame_id.split(':')
            task_id = int(parts[0])
            urgency = float(parts[1]) if len(parts) > 1 else 0.5
        except (ValueError, IndexError):
            task_id = 0
            urgency = 0.5

        self.pending_tasks[task_id] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'urgency': urgency,
        }
        self.get_logger().info(
            f'[swarm_manager] Task {task_id} urgency={urgency:.2f}')
        self._try_assign_pending()

    def _cb_formation(self, msg):
        f = msg.data.strip()
        if f in self.FORMATIONS:
            self.formation = f
            self.get_logger().info(f'[swarm_manager] Formation → {self.formation}')

    # ─── Main loop (10 Hz) ────────────────────────────────────────────────────

    def _manager_loop(self):
        dt  = 0.1
        tgt = self.waypoints[self.wp_idx]
        dx  = tgt[0] - self.centre_x
        dy  = tgt[1] - self.centre_y
        d   = math.hypot(dx, dy)

        if d < self.wp_tol:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
        else:
            step = min(self.max_speed * dt, d)
            self.centre_x += (dx / d) * step
            self.centre_y += (dy / d) * step

        offsets = self.FORMATIONS.get(self.formation, self.FORMATIONS['triangle'])

        for i in range(self.num_drones):
            ox, oy, oz = offsets[i] if i < len(offsets) else (0.0, 0.0, 0.0)
            msg = PoseStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.pose.position.x = self.centre_x + ox
            msg.pose.position.y = self.centre_y + oy
            msg.pose.position.z = BASE_ALT + oz   # ← staggered altitude
            self.formation_goal_pubs[i].publish(msg)

        cm = PoseStamped()
        cm.header.stamp    = self.get_clock().now().to_msg()
        cm.header.frame_id = 'world'
        cm.pose.position.x = self.centre_x
        cm.pose.position.y = self.centre_y
        cm.pose.position.z = BASE_ALT
        self.centre_pub.publish(cm)

    # ─── Triage-aware auction ─────────────────────────────────────────────────

    def _try_assign_pending(self):
        assigned = []
        sorted_tasks = sorted(
            self.pending_tasks.items(),
            key=lambda kv: kv[1]['urgency'],
            reverse=True,
        )

        for task_id, task in sorted_tasks:
            best_drone = None
            best_score = -1.0

            for i in range(self.num_drones):
                st = self.statuses[i]
                if st.get('state') not in ('patrol', 'idle'):
                    continue
                if st.get('battery', 0) < 15.0:
                    continue

                # 3-D distance for fair scoring
                px, py, pz = self.poses[i]
                dx   = task['x'] - px
                dy   = task['y'] - py
                dist = math.hypot(dx, dy) + 0.01   # XY distance for task assignment

                # Wind-adjusted distance — drone flying WITH wind is effectively closer
                dx_norm = dx / (dist + 0.01)
                dy_norm = dy / (dist + 0.01)
                wind_help = (WIND_X * dx_norm + WIND_Y * dy_norm)   # positive = wind helps
                eff_dist  = max(0.1, dist - wind_help * 0.4)

                # Payload penalty — loaded drone scores lower (slower)
                payload_pen = 0.15 if st.get('payload', False) else 0.0

                score = (0.5 * task['urgency'] +
                         0.4 * (1.0 / eff_dist) +
                         0.1 * (st.get('battery', 0.0) / 100.0) -
                         payload_pen)

                if score > best_score:
                    best_score = score
                    best_drone = i

            if best_drone is not None:
                self._dispatch(best_drone, task_id, task)
                self.active_tasks[task_id] = best_drone
                assigned.append(task_id)
                self.get_logger().info(
                    f'[swarm_manager] Task {task_id} '
                    f'(urgency={task["urgency"]:.2f}) → drone {best_drone}')

        for tid in assigned:
            del self.pending_tasks[tid]

    def _dispatch(self, drone_id, task_id, task):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = str(task_id)
        msg.pose.position.x = task['x']
        msg.pose.position.y = task['y']
        msg.pose.position.z = 0.5   # rescue altitude (drone_controller overrides anyway)
        self.task_goal_pubs[drone_id].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
