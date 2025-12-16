#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import math
import time
import ast

#Room entrances
ROOM1_ENTRY = [6.4, 10.66, 1.57] # Added Yaw
ROOM2_ENTRY = [-6.69, 10.66, 1.57]

#Room interior
ROOM1_POLY = [6.9, 8.5, 10.9, 12.2]
ROOM2_POLY = [-8.5, -6.9, 10.9, 12.2]

#Thresholds
ENTRY_THRESHOLD = 0.5
SCAN_DURATION = 4.0
SCAN_SPEED = 0.5 

#Topics
TOPIC_POSE        = "/go1_pose"
TOPIC_CMD         = "/cmd_vel"
TOPIC_PLANNER_GOAL = "/goal_pose"
TOPIC_DET_LABELS  = "/detections/labels" 
TOPIC_SPEECH      = "/robot_dog/speech"

class Mission5EmptyRoom(Node):
    def __init__(self):
        super().__init__("mission_5_emptyroom")

        self.create_subscription(PoseStamped, TOPIC_POSE, self.pose_cb, 10)
        self.create_subscription(String, TOPIC_DET_LABELS, self.det_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, TOPIC_CMD, 10)
        self.goal_pub = self.create_publisher(PoseStamped, TOPIC_PLANNER_GOAL, 10)
        self.speech_pub = self.create_publisher(String, TOPIC_SPEECH, 10)

        self.x = None
        self.y = None
        self.yaw = None

        self.has_stop_sign = False
        self.detections = []

        self.state = "GO_R1"
        self.scan_start = None
        self.goal_sent = False 

        self.get_logger().info("Mission 5 started: Finding the Empty Room...")

        self.create_timer(0.1, self.control_loop)

    def pose_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

        q = msg.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    def det_cb(self, msg):
        try:
            #Parse list
            self.detections = ast.literal_eval(msg.data)
        except Exception:
            self.detections = []

    def stop(self):
        self.cmd_pub.publish(Twist())

    def rotate_scan(self):
        # Direct Velocity Control for Rotation
        cmd = Twist()
        cmd.angular.z = SCAN_SPEED
        self.cmd_pub.publish(cmd)

    def send_planner_goal(self, x, y, yaw):
        if self.goal_sent: return
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(goal)
        self.goal_sent = True

    def check_arrival(self, tx, ty):
        if self.x is None: return False
        dist = math.sqrt((tx - self.x)**2 + (ty - self.y)**2)
        return dist < ENTRY_THRESHOLD

    def control_loop(self):
        if self.x is None: return

        #Go to room 1
        if self.state == "GO_R1":
            self.send_planner_goal(ROOM1_ENTRY[0], ROOM1_ENTRY[1], ROOM1_ENTRY[2])
            if self.check_arrival(ROOM1_ENTRY[0], ROOM1_ENTRY[1]):
                self.stop()
                self.scan_start = time.time()
                self.has_stop_sign = False
                self.state = "SCAN_R1"
                self.goal_sent = False
                self.get_logger().info("Arrived at Room 1. Scanning for Stop Sign...")
            return
        #Scan room 1
        if self.state == "SCAN_R1":
            self.rotate_scan()

            if "stopsign" in self.detections:
                self.has_stop_sign = True
                self.get_logger().warn("SAW STOP SIGN!")
            if time.time() - self.scan_start > SCAN_DURATION:
                self.stop()
                if not self.has_stop_sign:
                    self.state = "ENTER_R1"
                    self.goal_sent = False
                    self.get_logger().info("Room 1 is CLEAR. Entering...")
                else:
                    self.state = "GO_R2"
                    self.goal_sent = False
                    self.get_logger().info("Room 1 BLOCKED. Going to Room 2...")
            return

        #Go to room 2
        if self.state == "GO_R2":
            self.send_planner_goal(ROOM2_ENTRY[0], ROOM2_ENTRY[1], ROOM2_ENTRY[2])

            if self.check_arrival(ROOM2_ENTRY[0], ROOM2_ENTRY[1]):
                self.stop()
                self.scan_start = time.time()
                self.has_stop_sign = False
                self.state = "SCAN_R2"
                self.goal_sent = False
                self.get_logger().info("Arrived at Room 2. Scanning...")
            return

        #Scan room 2
        if self.state == "SCAN_R2":
            self.rotate_scan()

            if "stopsign" in self.detections:
                self.has_stop_sign = True
            if time.time() - self.scan_start > SCAN_DURATION:
                self.stop()
                if not self.has_stop_sign:
                    self.state = "ENTER_R2"
                    self.goal_sent = False
                    self.get_logger().info("Room 2 is CLEAR. Entering...")
                else:
                    self.state = "FAIL"
                    self.get_logger().error("Both rooms blocked? Mission Failed.")
            return

        #Enter room
        if self.state in ["ENTER_R1", "ENTER_R2"]:
            box = ROOM1_POLY if self.state == "ENTER_R1" else ROOM2_POLY
            tx = 0.5 * (box[0] + box[1])
            ty = 0.5 * (box[2] + box[3])
            self.send_planner_goal(tx, ty, 0.0)
            if box[0] <= self.x <= box[1] and box[2] <= self.y <= box[3]:
                self.stop()
                self.speech_pub.publish(String(data="bark"))
                self.state = "SUCCESS"
                self.get_logger().info("Mission 5 SUCCESS: Inside the Empty Room!")
            return

def main():
    rclpy.init()
    node = Mission5EmptyRoom()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()