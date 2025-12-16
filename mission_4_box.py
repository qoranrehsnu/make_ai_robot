#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math

# ================= CONFIG =================

SEARCH_POINTS = {
    'LEFT':   (-3.0, 10.0, 1.57),
    'CENTER': (1.0,  9.0,  1.57),
    'RIGHT':  (3.0, 10.0,  1.57)
}

PUSH_READY_POSES = {
    'LEFT':   (-3.0, 12.0, 0.0),
    'CENTER': (0.2,  15.0, -1.57),
    'RIGHT':  (3.0,  12.0, 3.14)
}

GOAL_ZONE_POSES = {
    'LEFT':   (-0.6, 12.0, 0.0),
    'CENTER': (-0.2, 12.5, -1.57),
    'RIGHT':  (0.6,  12.0, 3.14)
}

SEARCH_ORDER = ["LEFT", "CENTER", "RIGHT"]

ARRIVAL_THRESH = 0.6

TOPIC_POSE   = "/go1_pose"
TOPIC_GOAL   = "/goal_pose"
TOPIC_SPEECH = "/robot_dog/speech"

# ==========================================


class Mission4PushBox(Node):
    def __init__(self):
        super().__init__("mission_4_box")

        self.create_subscription(PoseStamped, TOPIC_POSE, self.pose_cb, 10)

        self.goal_pub = self.create_publisher(PoseStamped, TOPIC_GOAL, 10)
        self.speech_pub = self.create_publisher(String, TOPIC_SPEECH, 10)

        self.x = None
        self.y = None

        self.search_idx = 0
        self.box_side = None
        self.state = "SEARCH"
        self.last_goal = None

        self.get_logger().info("Mission 4 started (A* full navigation)")

        self.create_timer(0.2, self.control_loop)

    # ---------------- Callbacks ----------------
    def pose_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

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

        # ---------- SEARCH ----------
        if self.state == "SEARCH":
            side = SEARCH_ORDER[self.search_idx]
            x, y, yaw = SEARCH_POINTS[side]

            self.publish_goal(x, y, yaw)

            if self.reached(x, y):
                self.box_side = side
                self.state = "GO_PUSH_READY"
                self.last_goal = None
                self.get_logger().info(f"Box found at {side}")
            return

        # ---------- GO PUSH READY ----------
        if self.state == "GO_PUSH_READY":
            x, y, yaw = PUSH_READY_POSES[self.box_side]
            self.publish_goal(x, y, yaw)

            if self.reached(x, y):
                self.state = "PUSH"
                self.last_goal = None
            return

        # ---------- PUSH (A* to goal zone) ----------
        if self.state == "PUSH":
            x, y, yaw = GOAL_ZONE_POSES[self.box_side]
            self.publish_goal(x, y, yaw)

            if self.reached(x, y):
                self.speech_pub.publish(String(data="bark"))
                self.state = "SUCCESS"
                self.get_logger().info("Mission 4 SUCCESS")
            return

        if self.state == "SUCCESS":
            return


def main():
    rclpy.init()
    node = Mission4PushBox()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

