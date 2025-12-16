#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import math
import json
import time

# ================= CONFIG =================

# Strategy 1: Small rooms first
STRATEGY_1 = [
    (-2.03, -15.32, -1.61), (2.74, -15.30, -1.51), (2.79, -8.65, -1.60),
    (-3.79, -9.45, -1.05), (-7.79, 0.94, -1.64), (7.81, 0.85, -1.63),
    (-8.22, 4.75, 3.14), (-8.73, 11.20, 3.14),
    (8.73, 11.20, 0.04), (8.22, 4.75, -0.01),
]

# Strategy 2: Closest / Large rooms first
STRATEGY_2 = [
    (-7.60, -26.70, -3.04), (-7.71, -26.06, -1.57),
    (-5.73, -23.17, 3.10), (-7.49, -22.97, 1.63), (-7.91, -19.64, 3.13),
    (-8.55, -16.80, 2.65),
    (-5.89, -9.75, 2.49), (-7.05, -8.96, 3.13), (-7.15, -8.93, 2.35),
    (-7.71, -8.77, 1.56),
    (-7.79, -6.46, 2.40), (-7.71, -5.95, 2.57), (-7.37, -2.51, 2.61),
    (5.55, -8.97, -0.01), (6.88, -8.87, 0.89), (7.79, -8.09, 1.78),
    (5.08, -23.15, 0.07), (8.61, -21.03, 1.59), (9.07, -20.75, 0.47),
    (9.08, -17.74, 0.49),
    (5.51, -25.14, -1.63), (6.76, -26.66, 0.13), (6.17, -26.55, -0.80),
    (4.74, -27.36, -2.09), (7.97, -27.44, -3.10), (-1.61, -27.10, -1.56),
]

# Choose strategy here
WAYPOINTS = STRATEGY_1

# Thresholds
POS_THRESH = 0.5
YAW_THRESH = 0.3

# Control
V_FWD = 0.35
K_YAW = 1.6

# Topics
TOPIC_POSE   = "/go1_pose"
TOPIC_CMD    = "/cmd_vel"
TOPIC_DET    = "/detections"
TOPIC_SPEECH = "/robot_dog/speech"

# Edible labels
EDIBLE_LABELS = {"banana", "apple", "pizza"}

# ==========================================


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Mission2FindFood(Node):
    def __init__(self):
        super().__init__("mission_2_food")

        self.create_subscription(PoseStamped, TOPIC_POSE, self.pose_cb, 10)
        self.create_subscription(String, TOPIC_DET, self.det_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, TOPIC_CMD, 10)
        self.speech_pub = self.create_publisher(String, TOPIC_SPEECH, 10)

        self.x = None
        self.y = None
        self.yaw = None

        self.detections = []

        self.wp_idx = 0
        self.state = "SEARCH"

        self.get_logger().info("Mission 2 started: find edible food")

        self.create_timer(0.05, self.control_loop)

    # ---------------- Callbacks ----------------
    def pose_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

        q = msg.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    def det_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.detections = data.get("detections", [])
        except Exception:
            self.detections = []

    # ---------------- Motion helpers ----------------
    def stop(self):
        self.cmd_pub.publish(Twist())

    def move_to(self, tx, ty, tyaw):
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        yaw_err = normalize_angle(tyaw - self.yaw)

        cmd = Twist()
        cmd.linear.x = V_FWD
        cmd.angular.z = K_YAW * yaw_err
        self.cmd_pub.publish(cmd)

        return dist, abs(yaw_err)

    def reached(self, tx, ty, tyaw):
        d = math.hypot(tx - self.x, ty - self.y)
        y = abs(normalize_angle(tyaw - self.yaw))
        return d < POS_THRESH and y < YAW_THRESH

    # ---------------- FSM ----------------
    def control_loop(self):
        if self.x is None:
            return

        # ---------- SEARCH ----------
        if self.state == "SEARCH":

            # Check food detection first (immediate success)
            for d in self.detections:
                if d.get("label") in EDIBLE_LABELS:
                    self.stop()
                    self.speech_pub.publish(String(data="bark"))
                    self.state = "SUCCESS"
                    self.get_logger().info(f"Edible food found: {d.get('label')}")
                    return

            # Move to next waypoint
            if self.wp_idx >= len(WAYPOINTS):
                self.wp_idx = 0  # loop search (safe default)

            tx, ty, tyaw = WAYPOINTS[self.wp_idx]
            self.move_to(tx, ty, tyaw)

            if self.reached(tx, ty, tyaw):
                self.wp_idx += 1
            return

        # ---------- SUCCESS ----------
        if self.state == "SUCCESS":
            self.stop()
            return


def main():
    rclpy.init()
    node = Mission2FindFood()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

