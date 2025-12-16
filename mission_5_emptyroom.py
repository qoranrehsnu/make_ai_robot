#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import math
import time
import json

# ================= CONFIG =================
# Room entrances
ROOM1_ENTRY = [6.69, 10.66]
ROOM2_ENTRY = [-6.69, 10.66]

# Room interior box regions [xmin, xmax, ymin, ymax]
ROOM1_POLY = [6.9, 8.5, 10.9, 12.2]
ROOM2_POLY = [-8.5, -6.9, 10.9, 12.2]

ENTRY_THRESHOLD = 0.6

SCAN_DURATION = 4.0
SCAN_SPEED = 0.5

V_FWD = 0.3
K_YAW = 1.6

TOPIC_POSE   = "/go1_pose"
TOPIC_CMD    = "/cmd_vel"
TOPIC_DET    = "/detections"
TOPIC_SPEECH = "/robot_dog/speech"
# ==========================================


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def point_in_room(x, y, box):
    xmin, xmax, ymin, ymax = box
    return xmin <= x <= xmax and ymin <= y <= ymax


class Mission5EmptyRoom(Node):
    def __init__(self):
        super().__init__("mission_5_emptyroom")

        self.create_subscription(PoseStamped, TOPIC_POSE, self.pose_cb, 10)
        self.create_subscription(String, TOPIC_DET, self.det_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, TOPIC_CMD, 10)
        self.speech_pub = self.create_publisher(String, TOPIC_SPEECH, 10)

        self.x = None
        self.y = None
        self.yaw = None

        self.has_stop_sign = False
        self.detections = []

        self.state = "GO_R1"
        self.scan_start = None
        self.target_room = None

        self.get_logger().info("Mission 5 started (minimal policy)")

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

    def rotate_scan(self):
        cmd = Twist()
        cmd.angular.z = SCAN_SPEED
        self.cmd_pub.publish(cmd)

    def move_to(self, tx, ty):
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)
        yaw_err = normalize_angle(target_yaw - self.yaw)

        cmd = Twist()
        cmd.linear.x = V_FWD
        cmd.angular.z = K_YAW * yaw_err
        self.cmd_pub.publish(cmd)

        return dist

    # ---------------- FSM ----------------
    def control_loop(self):
        if self.x is None:
            return

        # ---------- GO TO ROOM 1 ----------
        if self.state == "GO_R1":
            d = self.move_to(ROOM1_ENTRY[0], ROOM1_ENTRY[1])
            if d < ENTRY_THRESHOLD:
                self.stop()
                self.scan_start = time.time()
                self.has_stop_sign = False
                self.state = "SCAN_R1"
                self.get_logger().info("At Room 1 entrance → scanning")
            return

        # ---------- SCAN ROOM 1 ----------
        if self.state == "SCAN_R1":
            self.rotate_scan()

            if any(d.get("label") == "stopsign" for d in self.detections):
                self.has_stop_sign = True

            if time.time() - self.scan_start > SCAN_DURATION:
                self.stop()
                if not self.has_stop_sign:
                    self.target_room = 1
                    self.state = "ENTER_R1"
                    self.get_logger().info("Room 1 has NO stop sign → entering")
                else:
                    self.state = "GO_R2"
                    self.get_logger().info("Room 1 blocked → checking Room 2")
            return

        # ---------- GO TO ROOM 2 ----------
        if self.state == "GO_R2":
            d = self.move_to(ROOM2_ENTRY[0], ROOM2_ENTRY[1])
            if d < ENTRY_THRESHOLD:
                self.stop()
                self.scan_start = time.time()
                self.has_stop_sign = False
                self.state = "SCAN_R2"
                self.get_logger().info("At Room 2 entrance → scanning")
            return

        # ---------- SCAN ROOM 2 ----------
        if self.state == "SCAN_R2":
            self.rotate_scan()

            if any(d.get("label") == "stopsign" for d in self.detections):
                self.has_stop_sign = True

            if time.time() - self.scan_start > SCAN_DURATION:
                self.stop()
                if not self.has_stop_sign:
                    self.target_room = 2
                    self.state = "ENTER_R2"
                    self.get_logger().info("Room 2 has NO stop sign → entering")
                else:
                    self.state = "FAIL"
                    self.get_logger().error("Both rooms blocked → FAIL")
            return

        # ---------- ENTER ROOM ----------
        if self.state in ["ENTER_R1", "ENTER_R2"]:
            box = ROOM1_POLY if self.state == "ENTER_R1" else ROOM2_POLY
            tx = 0.5 * (box[0] + box[1])
            ty = 0.5 * (box[2] + box[3])

            self.move_to(tx, ty)

            if point_in_room(self.x, self.y, box):
                self.stop()
                self.speech_pub.publish(String(data="bark"))
                self.state = "SUCCESS"
                self.get_logger().info("Mission 5 SUCCESS")
            return

        # ---------- END STATES ----------
        if self.state in ["SUCCESS", "FAIL"]:
            self.stop()
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

