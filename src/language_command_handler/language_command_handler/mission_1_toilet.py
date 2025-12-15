#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math

# ★★★ CONFIGURATION ★★★
TOILET_X = 5.0   
TOILET_Y = 2.0
TOILET_YAW = 1.57 # 90 degrees
STOP_DIST = 0.5   
STOP_YAW = 0.2    

# Topics (Update if needed)
TOPIC_ROBOT_POSE = '/go1_pose'          
TOPIC_PLANNER_GOAL = '/goal_pose'
TOPIC_SPEECH = '/robot_dog/speech'      

class ToiletMission(Node):
    def __init__(self):
        super().__init__('mission_1_toilet')
        
        self.create_subscription(PoseStamped, TOPIC_ROBOT_POSE, self.pose_cb, 10)
        self.goal_pub = self.create_publisher(PoseStamped, TOPIC_PLANNER_GOAL, 10)
        self.speech_pub = self.create_publisher(String, TOPIC_SPEECH, 10)
        
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.goal_sent = False

        self.get_logger().info(f"Mission 1 Started: Heading to Toilet ({TOILET_X}, {TOILET_Y})")
        
        self.create_timer(0.5, self.control_loop)
        self.create_timer(2.0, self.send_goal_command)

    def pose_cb(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def send_goal_command(self):
        if self.goal_sent:
            return
            
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = TOILET_X
        goal.pose.position.y = TOILET_Y
        goal.pose.orientation.z = math.sin(TOILET_YAW / 2.0)
        goal.pose.orientation.w = math.cos(TOILET_YAW / 2.0)

        self.goal_pub.publish(goal)
        self.get_logger().info(f"Sent Goal to Planner: {TOILET_X}, {TOILET_Y}")
        self.goal_sent = True

    def control_loop(self):
        if self.current_x is None:
            return

        dist = math.sqrt((TOILET_X - self.current_x)**2 + (TOILET_Y - self.current_y)**2)
        yaw_diff = abs(TOILET_YAW - self.current_yaw)
        if yaw_diff > math.pi:
            yaw_diff = 2*math.pi - yaw_diff

        if dist < STOP_DIST:
            if yaw_diff < STOP_YAW:
                self.get_logger().info("Arrived! Barking...")
                msg = String()
                msg.data = "bark"
                self.speech_pub.publish(msg)
                raise SystemExit
            else:
                self.get_logger().info(f"Near target, aligning... (Yaw Error: {yaw_diff:.2f})")

def main():
    rclpy.init()
    node = ToiletMission()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()