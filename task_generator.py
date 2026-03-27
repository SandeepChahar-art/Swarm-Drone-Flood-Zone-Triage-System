#!/usr/bin/env python3
"""
task_generator.py — Flood Zone Triage with Survival Probability
(3-D version — tasks spawned at ground level z=0)
"""
import rclpy
from rclpy.node import Node
import random, json, time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

FLOOD_ZONES = [
    {"name":"Market Area",    "x":(1.0,4.0), "y":(1.0,5.0),  "water_base":0.85, "spawn_rate":0.6},
    {"name":"Residential",    "x":(5.0,9.0), "y":(1.0,6.0),  "water_base":0.55, "spawn_rate":0.4},
    {"name":"Low-lying Area", "x":(1.5,4.5), "y":(7.0,13.0), "water_base":0.92, "spawn_rate":0.7},
    {"name":"Main Road",      "x":(4.0,8.0), "y":(6.0,10.0), "water_base":0.45, "spawn_rate":0.3},
]

PERSON_PROFILES = [
    {"name":"Elderly Woman",      "age_group":"65+",   "condition":0.92, "description":"Elderly, cannot swim, diabetic",           "label":"elderly",  "base_survival":0.35},
    {"name":"Infant",             "age_group":"0-2",   "condition":0.95, "description":"Infant, held by parent on rooftop",         "label":"infant",   "base_survival":0.30},
    {"name":"Injured Man",        "age_group":"30-50", "condition":0.82, "description":"Leg fracture, unable to move",              "label":"injured",  "base_survival":0.50},
    {"name":"Child",              "age_group":"5-12",  "condition":0.75, "description":"Child, separated from family",              "label":"child",    "base_survival":0.45},
    {"name":"Pregnant Woman",     "age_group":"25-35", "condition":0.88, "description":"Pregnant, 8 months, stranded on roof",      "label":"pregnant", "base_survival":0.40},
    {"name":"Healthy Adult",      "age_group":"20-40", "condition":0.22, "description":"Healthy adult, needs evacuation",           "label":"adult",    "base_survival":0.75},
    {"name":"Heart Patient",      "age_group":"55-70", "condition":0.90, "description":"Cardiac condition, no medication",          "label":"cardiac",  "base_survival":0.28},
    {"name":"Wheelchair User",    "age_group":"40-60", "condition":0.85, "description":"Wheelchair bound, cannot self-evacuate",    "label":"disabled", "base_survival":0.38},
    {"name":"Group of Children",  "age_group":"6-14",  "condition":0.70, "description":"3 children on school rooftop",             "label":"children", "base_survival":0.42},
    {"name":"Road Blockage",      "age_group":"N/A",   "condition":0.10, "description":"Submerged road blocking rescue vehicles",   "label":"road",     "base_survival":1.00},
]

# Water level label categories (for 3-D water height visualisation)
WATER_CATEGORIES = [
    (0.0,  0.25, "Ankle Deep",  0.25),
    (0.25, 0.50, "Knee Deep",   0.60),
    (0.50, 0.70, "Waist Deep",  1.00),
    (0.70, 0.85, "Chest Deep",  1.40),
    (0.85, 1.00, "Submerged",   1.80),
]

def water_category(level):
    for lo, hi, label, height in WATER_CATEGORIES:
        if lo <= level < hi:
            return label, height
    return "Submerged", 1.80

def compute_survival(base_survival, water_level, condition, elapsed_min):
    drop_rate    = condition * 0.025 + water_level * 0.015
    survival_now = max(0.05, base_survival - (drop_rate * elapsed_min))
    survival_rescued = min(0.97, survival_now + 0.45)
    return round(survival_now, 2), round(survival_rescued, 2)

def urgency_score(water, condition, elapsed_min, survival_now):
    time_f = min(elapsed_min / 120.0, 1.0)
    u = (0.4*water + 0.3*condition + 0.2*(1.0 - survival_now) + 0.1*time_f)
    return round(min(u, 1.0), 3)

def priority_label(urgency):
    if urgency >= 0.75: return "CRITICAL"
    if urgency >= 0.50: return "HIGH"
    if urgency >= 0.30: return "MEDIUM"
    return "LOW"


class FloodTaskGenerator(Node):
    def __init__(self):
        super().__init__('task_generator')
        self.declare_parameter('spawn_interval', 8.0)
        self.declare_parameter('max_tasks',      20)
        self.interval   = self.get_parameter('spawn_interval').value
        self.max_tasks  = self.get_parameter('max_tasks').value
        self.count      = 0
        self.start_time = time.time()
        self.task_pub   = self.create_publisher(PoseStamped, '/swarm/detected_task', 10)
        self.triage_pub = self.create_publisher(String,      '/swarm/triage_data',   10)
        self.create_timer(self.interval, self._spawn)
        self.get_logger().info('[task_generator] 3D Flood Triage Online')

    def _spawn(self):
        if self.count >= self.max_tasks:
            return

        zone    = random.choices(FLOOD_ZONES, weights=[z['spawn_rate'] for z in FLOOD_ZONES])[0]
        profile = random.choice(PERSON_PROFILES)
        tx      = round(random.uniform(*zone['x']), 2)
        ty      = round(random.uniform(*zone['y']), 2)
        water   = round(min(zone['water_base'] + random.uniform(-0.1, 0.15), 1.0), 2)
        cond    = round(min(profile['condition'] + random.uniform(-0.03, 0.03), 1.0), 2)
        elapsed = (time.time() - self.start_time) / 60.0

        surv_now, surv_rescued = compute_survival(profile['base_survival'], water, cond, elapsed)
        urgency  = urgency_score(water, cond, elapsed, surv_now)
        priority = priority_label(urgency)
        wcat, wheight = water_category(water)
        label    = f"{profile['label']}_{self.count}"
        tid      = self.count
        self.count += 1

        # PoseStamped — z=0 (ground level task)
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{tid}:{urgency}"
        msg.pose.position.x = float(tx)
        msg.pose.position.y = float(ty)
        msg.pose.position.z = 0.0   # tasks are on the ground
        self.task_pub.publish(msg)

        # Full triage JSON for visualizer
        triage = {
            "task_id":             tid,
            "label":               label,
            "person_name":         profile['name'],
            "age_group":           profile['age_group'],
            "description":         profile['description'],
            "x": tx, "y": ty, "z": 0.0,
            "zone":                zone['name'],
            "water_level":         water,
            "water_category":      wcat,
            "water_height_m":      wheight,   # metres for 3-D visualisation
            "condition":           cond,
            "survival_now":        surv_now,
            "survival_if_rescued": surv_rescued,
            "urgency":             urgency,
            "priority":            priority,
            "state":               "pending",
            "spawn_time":          round(elapsed, 1),
        }
        t = String()
        t.data = json.dumps(triage)
        self.triage_pub.publish(t)

        self.get_logger().info(
            f'[task_gen] {profile["name"]} ({wcat}) | '
            f'surv={surv_now*100:.0f}%→{surv_rescued*100:.0f}% | {priority}')


def main(args=None):
    rclpy.init(args=args)
    node = FloodTaskGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
