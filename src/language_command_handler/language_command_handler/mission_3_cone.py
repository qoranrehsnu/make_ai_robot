import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
import math

# ★★★ 여기서 콘 좌표 수정하세요 ★★★
CONES = {
    'red':   (3.0, 1.0),
    'blue':  (3.0, 0.0),
    'green': (3.0, -1.0)
}

# 기본 목표 (나중에 "red", "blue" 등으로 바꾸세요)
TARGET_COLOR = 'red' 

from language_command_handler.astar_planner import AStarPlanner

class ConeMission(Node):
    def __init__(self):
        super().__init__('mission_3_cone')
        
        self.create_subscription(PoseStamped, '/go1_pose', self.pose_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        
        self.path_pub = self.create_publisher(Path, '/local_path', 10)
        self.speech_pub = self.create_publisher(String, '/robot_dog/speech', 10)
        
        self.planner = AStarPlanner()
        self.current_pose = None
        self.map_received = False
        
        # 목표 좌표 설정
        if TARGET_COLOR in CONES:
            self.target_x, self.target_y = CONES[TARGET_COLOR]
            self.get_logger().info(f"Target: {TARGET_COLOR} Cone at ({self.target_x}, {self.target_y})")
        else:
            self.get_logger().error("Unknown Color!")
            raise SystemExit

        self.timer = self.create_timer(1.0, self.control_loop)

    def pose_cb(self, msg):
        self.current_pose = msg

    def map_cb(self, msg):
        self.planner.update_map(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin)
        self.map_received = True

    def control_loop(self):
        if self.current_pose is None or not self.map_received:
            return

        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y
        dist = math.sqrt((self.target_x - curr_x)**2 + (self.target_y - curr_y)**2)

        if dist < 0.5: # 0.5m 이내 도착
            self.get_logger().info(f"Arrived at {TARGET_COLOR} cone!")
            self.path_pub.publish(Path(header=self.current_pose.header)) # 정지
            
            msg = String()
            msg.data = "bark"
            self.speech_pub.publish(msg)
            raise SystemExit

        # 경로 계획 및 발행
        path_list = self.planner.plan((curr_x, curr_y), (self.target_x, self.target_y))
        if path_list:
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            for (wx, wy) in path_list:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = wx
                pose.pose.position.y = wy
                path_msg.poses.append(pose)
            self.path_pub.publish(path_msg)

def main():
    rclpy.init()
    try:
        rclpy.spin(ConeMission())
    except SystemExit:
        rclpy.shutdown()

if __name__ == '__main__':
    main()