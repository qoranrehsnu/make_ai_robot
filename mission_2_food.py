#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import json

# ================= CONFIG =================

# Strategy 1: Small rooms first
WAYPOINTS = [
    (8.73, 11.20, 0.04),(8.22, 4.75, -0.01), (7.81, 0.85, -1.63), (-2.03, -15.32, -1.61), (2.74, -15.30, -1.51), (2.79, -8.65, -1.60),
    (-3.79, -9.45, -1.05), (-7.79, 0.94, -1.64), 
    (-8.22, 4.75, 3.14), (-8.73, 11.20, 3.14)
     
]

ARRIVAL_THRESH = 0.6

EDIBLE_LABELS = {"banana", "apple", "pizza"}

TOPIC_POSE   = "/go1_pose"
TOPIC_GOAL   = "/goal_pose"
TOPIC_DET    = "/detections"
TOPIC_SPEECH = "/robot_dog/speech"

# ==========================================


class Mission2FindFood(Node):
    def __init__(self):
        super().__init__("mission_2_food")

        self.create_subscription(PoseStamped, TOPIC_POSE, self.pose_cb, 10)
        self.create_subscription(String, TOPIC_DET, self.det_cb, 10)

        self.goal_pub = self.create_publisher(PoseStamped, TOPIC_GOAL, 10)
        self.speech_pub = self.create_publisher(String, TOPIC_SPEECH, 10)

        self.x = None
        self.y = None
        self.detections = []

        self.wp_idx = 0
        self.state = "GO_WP"
        self.last_goal = None

        self.get_logger().info("Mission 2 started (A* full navigation)")

        self.create_timer(0.2, self.control_loop)

    # ---------------- Callbacks ----------------
    def pose_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

    def det_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.detections = data.get("detections", [])
        except Exception:
            self.detections = []

    # ---------------- Helpers ----------------
    def publish_goal(self, x, y, yaw):
        if self.last_goal == (x, y, yaw):
            return

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pub.publish(goal)
        self.last_goal = (x, y, yaw)

    def reached(self, x, y):
        if self.x is None:
            return False
        return math.hypot(self.x - x, self.y - y) < ARRIVAL_THRESH

    # ---------------- FSM ----------------
    def control_loop(self):
        if self.x is None:
            return

        # Food detection has priority
        for d in self.detections:
            if d.get("label") in EDIBLE_LABELS:
                self.speech_pub.publish(String(data="bark"))
                self.state = "SUCCESS"
                self.get_logger().info(f"Food found: {d.get('label')}")
                return

        if self.state == "GO_WP":
            if self.wp_idx >= len(WAYPOINTS):
                self.wp_idx = 0  # loop safely

            x, y, yaw = WAYPOINTS[self.wp_idx]
            self.publish_goal(x, y, yaw)

            if self.reached(x, y):
                self.last_goal = None
                self.wp_idx += 1
            return

        if self.state == "SUCCESS":
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

