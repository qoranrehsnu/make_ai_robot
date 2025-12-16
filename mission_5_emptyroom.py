#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import time
import json

# ================= CONFIG =================

# Hard-coded waiting point
WAIT_POINT = (5.0, 9.0, 1.57)

ROOM1_ENTRY = (6.69, 10.66, 1.57)
ROOM2_ENTRY = (-6.69, 10.66, 1.57)

ROOM1_CENTER = (7.7, 11.5, 0.0)
ROOM2_CENTER = (-7.7, 11.5, 3.14)

ENTRY_THRESHOLD = 0.6
SCAN_DURATION = 4.0

TOPIC_POSE   = "/go1_pose"
TOPIC_GOAL   = "/goal_pose"
TOPIC_DET    = "/detections"
TOPIC_SPEECH = "/robot_dog/speech"

# ==========================================


class Mission5EmptyRoom(Node):
    def __init__(self):
        super().__init__("mission_5_emptyroom")

        self.create_subscription(PoseStamped, TOPIC_POSE, self.pose_cb, 10)
        self.create_subscription(String, TOPIC_DET, self.det_cb, 10)

        self.goal_pub = self.create_publisher(PoseStamped, TOPIC_GOAL, 10)
        self.speech_pub = self.create_publisher(String, TOPIC_SPEECH, 10)

        self.x = None
        self.y = None

        self.has_stop_sign = False
        self.detections = []

        # FSM starts here
        self.state = "GO_WAIT"
        self.scan_start = None
        self.last_goal = None

        self.get_logger().info("Mission 5 started (wait point → empty room)")

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

    def reached(self, tx, ty):
        if self.x is None:
            return False
        return math.hypot(tx - self.x, ty - self.y) < ENTRY_THRESHOLD

    # ---------------- FSM ----------------

    def control_loop(self):
        if self.x is None:
            return

        # ---------- GO TO WAIT POINT ----------
        if self.state == "GO_WAIT":
            self.publish_goal(*WAIT_POINT)

            if self.reached(WAIT_POINT[0], WAIT_POINT[1]):
                self.state = "GO_R1"
                self.last_goal = None
                self.get_logger().info("Reached wait point → start Mission 5")
            return

        # ---------- GO TO ROOM 1 ----------
        if self.state == "GO_R1":
            self.publish_goal(*ROOM1_ENTRY)

            if self.reached(ROOM1_ENTRY[0], ROOM1_ENTRY[1]):
                self.scan_start = time.time()
                self.has_stop_sign = False
                self.state = "SCAN_R1"
            return

        # ---------- SCAN ROOM 1 ----------
        if self.state == "SCAN_R1":
            if any(d.get("label") == "stopsign" for d in self.detections):
                self.has_stop_sign = True

            if time.time() - self.scan_start > SCAN_DURATION:
                self.last_goal = None
                if not self.has_stop_sign:
                    self.state = "ENTER_R1"
                else:
                    self.state = "GO_R2"
            return

        # ---------- GO TO ROOM 2 ----------
        if self.state == "GO_R2":
            self.publish_goal(*ROOM2_ENTRY)

            if self.reached(ROOM2_ENTRY[0], ROOM2_ENTRY[1]):
                self.scan_start = time.time()
                self.has_stop_sign = False
                self.state = "SCAN_R2"
            return

        # ---------- SCAN ROOM 2 ----------
        if self.state == "SCAN_R2":
            if any(d.get("label") == "stopsign" for d in self.detections):
                self.has_stop_sign = True

            if time.time() - self.scan_start > SCAN_DURATION:
                self.last_goal = None
                if not self.has_stop_sign:
                    self.state = "ENTER_R2"
                else:
                    self.state = "FAIL"
            return

        # ---------- ENTER ROOM ----------
        if self.state == "ENTER_R1":
            self.publish_goal(*ROOM1_CENTER)
            if self.reached(ROOM1_CENTER[0], ROOM1_CENTER[1]):
                self.speech_pub.publish(String(data="bark"))
                self.state = "SUCCESS"
            return

        if self.state == "ENTER_R2":
            self.publish_goal(*ROOM2_CENTER)
            if self.reached(ROOM2_CENTER[0], ROOM2_CENTER[1]):
                self.speech_pub.publish(String(data="bark"))
                self.state = "SUCCESS"
            return

        # ---------- END ----------
        if self.state in ["SUCCESS", "FAIL"]:
            return


def main():
    rclpy.init()
    node = Mission5EmptyRoom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

