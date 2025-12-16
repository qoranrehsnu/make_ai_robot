#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32
import math
import ast # For parsing "['nurse']" strings safety

#Target Room
NURSE_ROOM_X = -8.5
NURSE_ROOM_Y = -27.25

#Distances
APPROACH_DIST = 2.0
TARGET_DIST = 1.2
MAX_LIN_SPEED = 0.35
MAX_ANG_SPEED = 0.6

#Circling
CIRCLE_RADIUS = 1.2
CIRCLE_SPEED_MPS = 0.4

#Topics
TOPIC_POSE        = "/go1_pose"
TOPIC_CMD         = "/cmd_vel"
TOPIC_PLANNER_GOAL = "/goal_pose"
TOPIC_DET_LABELS  = "/detections/labels"
TOPIC_DET_DIST    = "/detections/distance"
TOPIC_SPEECH      = "/robot_dog/speech"

class NurseMission(Node):
    def __init__(self):
        super().__init__('mission_6_nurse')

        # Subscribers
        self.create_subscription(PoseStamped, TOPIC_POSE, self.pose_cb, 10)
        self.create_subscription(String, TOPIC_DET_LABELS, self.labels_cb, 10)
        self.create_subscription(Float32, TOPIC_DET_DIST, self.dist_cb, 10)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, TOPIC_PLANNER_GOAL, 10)
        self.vel_pub = self.create_publisher(Twist, TOPIC_CMD, 10)
        self.speech_pub = self.create_publisher(String, TOPIC_SPEECH, 10)

        # State Variables
        self.current_x = None
        self.current_y = None
        
        # Perception State
        self.nurse_seen = False
        self.nurse_dist = 99.9

        # Mission State
        self.state = "GO_ROOM" 
        self.goal_sent = False
        self.circle_start_time = None

        self.get_logger().info(f"Mission 6 Started: Going to Nurse Room ({NURSE_ROOM_X}, {NURSE_ROOM_Y})")
        self.create_timer(0.1, self.control_loop)

    def pose_cb(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

    def labels_cb(self, msg):
        try:
            labels = ast.literal_eval(msg.data)
            self.nurse_seen = any(label in ['nurse', 'person'] for label in labels)
        except:
            self.nurse_seen = False

    def dist_cb(self, msg):
        if msg.data > 0:
            self.nurse_dist = float(msg.data)

    def stop_robot(self):
        self.vel_pub.publish(Twist())

    def send_planner_goal(self):
        if self.goal_sent: return
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = NURSE_ROOM_X
        goal.pose.position.y = NURSE_ROOM_Y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.goal_sent = True
        self.get_logger().info("Sent Goal to Planner -> Moving to Room")

    def control_loop(self):
        if self.current_x is None: return

        #Move to nurse room
        if self.state == "GO_ROOM":
            self.send_planner_goal()
            dist = math.hypot(NURSE_ROOM_X - self.current_x, NURSE_ROOM_Y - self.current_y)
            if dist < APPROACH_DIST:
                self.get_logger().info(f"Arrived near room ({dist:.2f}m). Searching for Nurse...")
                self.stop_robot()
                self.state = "FIND_NURSE"
        #find nurse
        elif self.state == "FIND_NURSE":
            if self.nurse_seen:
                self.get_logger().info("Nurse Detected! Approaching...")
                self.stop_robot()
                self.state = "APPROACH"
            else:
                cmd = Twist()
                cmd.angular.z = 0.4 
                self.vel_pub.publish(cmd)

        #3. Approach the nurse
        elif self.state == "APPROACH":
            if not self.nurse_seen:
                self.get_logger().warn("Lost sight of nurse. Stopping.")
                self.stop_robot()
                self.state = "FIND_NURSE"
                return
            if self.nurse_dist <= TARGET_DIST:
                self.get_logger().info(f"Close enough ({self.nurse_dist:.2f}m). Starting Circle...")
                self.stop_robot()
                self.state = "CIRCLE"
            else:
                cmd = Twist()
                cmd.linear.x = min(0.3, MAX_LIN_SPEED)
                self.vel_pub.publish(cmd)

        #Move around the target
        elif self.state == "CIRCLE":
            if self.circle_start_time is None:
                self.circle_start_time = self.get_clock().now().nanoseconds / 1e9
                self.speech_pub.publish(String(data="bark"))
                return
            linear_v = CIRCLE_SPEED_MPS
            angular_w = linear_v / CIRCLE_RADIUS
            duration = (2 * math.pi) / angular_w
            
            elapsed = (self.get_clock().now().nanoseconds / 1e9) - self.circle_start_time
            
            if elapsed < duration:
                cmd = Twist()
                cmd.linear.x = linear_v
                cmd.angular.z = angular_w
                self.vel_pub.publish(cmd)
            else:
                self.get_logger().info("Circle Complete. Mission Done.")
                self.stop_robot()
                self.state = "DONE"

        elif self.state == "DONE":
            self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    node = NurseMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()