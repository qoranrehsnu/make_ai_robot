#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import subprocess
import time

# ★★★ SEARCH LOCATIONS (The 3 slots where cones might be) ★★★
SEARCH_SPOTS = [
    (1.2, 15.4, 0.785),   # Spot A (Top)
    (1.2, 15.4, 1.57),   # Spot B (Middle)
    (1.2, 15.4, 2.355)   # Spot C (Bottom)
]
STOP_DIST = 0.5
SEARCH_WAIT_TIME = 2.0  # How long to wait/stare at a cone before giving up

# Topics
TOPIC_ROBOT_POSE = '/go1_pose'          
TOPIC_PLANNER_GOAL = '/goal_pose'
TOPIC_SPEECH = '/robot_dog/speech'
# TODO: Update this to your actual perception topic!
TOPIC_DETECTIONS = '/detections/labels' 

def set_perception_targets(targets):
    """
    Sets the target_classes parameter of the perception_node dynamically.
    """
    param_str = str(targets).replace("'", '"')
    cmd = ["ros2", "param", "set", "/perception_node", "target_classes", param_str]
    try:
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
        print(f"Perception node now targeting: {targets}")
    except subprocess.CalledProcessError:
        print("Failed to set perception targets")

class ConeMission(Node):
    def __init__(self):
        super().__init__('mission_3_cone')
        
        # 1. Get Target Color from LLM (default: red)
        self.declare_parameter('target_color', 'redcone')
        self.target_color = self.get_parameter('target_color').get_parameter_value().string_value
        
        # 2. Configure Perception to ONLY look for that color
        # This simplifies logic: If we see ANYTHING, it must be the right color.
        set_perception_targets([self.target_color])
        
        # 3. State Management
        self.spot_index = 0  # Which spot are we checking?
        self.state = "MOVING" # MOVING -> SCANNING -> FOUND
        self.scan_start_time = 0
        self.object_detected = False

        # 4. ROS Setup
        self.create_subscription(PoseStamped, TOPIC_ROBOT_POSE, self.pose_cb, 10)
        
        # TODO: Change 'String' to the actual message type of your perception node (e.g., Detection2DArray)
        self.create_subscription(String, TOPIC_DETECTIONS, self.detection_cb, 10)
        
        self.goal_pub = self.create_publisher(PoseStamped, TOPIC_PLANNER_GOAL, 10)
        self.speech_pub = self.create_publisher(String, TOPIC_SPEECH, 10)
        
        self.current_x = None
        self.current_y = None

        self.get_logger().info(f"Mission 3 Started: Searching for {self.target_color.upper()} cone...")
        
        self.create_timer(0.5, self.control_loop)
        self.create_timer(2.0, self.send_goal_command)

    def pose_cb(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

    def detection_cb(self, msg):
        """
        Callback when the camera sees something.
        Since we set the filter to ONLY 'red' (or blue/green), 
        any detection implies we found it.
        """
        # If msg.data is not empty, we saw it!
        # logic depends on your message type. Assuming String here:
        if self.target_color in msg.data.lower(): 
            self.object_detected = True

    def send_goal_command(self):
        if self.state != "MOVING": return
        
        # Go to the current search spot
        target_x, target_y, target_yaw = SEARCH_SPOTS[self.spot_index]
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = target_x
        goal.pose.position.y = target_y
        goal.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal.pose.orientation.w = math.cos(target_yaw / 2.0)
        self.goal_pub.publish(goal)

    def control_loop(self):
        if self.current_x is None: return
        
        # 1. Check if we found it (Priority)
        if self.object_detected:
            self.get_logger().info(f"FOUND THE {self.target_color} CONE! Barking...")
            self.speech_pub.publish(String(data="bark"))
            self.goal_pub.publish(PoseStamped(header=self.get_clock().now().to_msg())) # Stop goal (optional)
            raise SystemExit

        # 2. Logic for moving between spots
        target_x, target_y, _ = SEARCH_SPOTS[self.spot_index]
        dist = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
        
        if self.state == "MOVING":
            if dist < STOP_DIST:
                self.get_logger().info(f"Arrived at Spot {self.spot_index+1}. Scanning...")
                self.state = "SCANNING"
                self.scan_start_time = time.time()
                self.object_detected = False # Reset detection for this new look

        elif self.state == "SCANNING":
            # Wait for a few seconds to let the camera see
            if time.time() - self.scan_start_time > SEARCH_WAIT_TIME:
                self.get_logger().info(f"Not found at Spot {self.spot_index+1}. Moving to next...")
                self.spot_index += 1
                
                if self.spot_index >= len(SEARCH_SPOTS):
                    self.get_logger().error("Searched all spots but could not find the cone!")
                    raise SystemExit
                
                self.state = "MOVING"

def main():
    rclpy.init()
    node = ConeMission()
    try: rclpy.spin(node)
    except SystemExit: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()