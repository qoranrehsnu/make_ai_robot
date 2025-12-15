import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
import math
import time

# ★★★ 간호사 위치 ★★★
NURSE_X = 8.0
NURSE_Y = 8.0
APPROACH_DIST = 1.0  # 간호사 1m 앞까지만 접근

from language_command_handler.astar_planner import AStarPlanner

class NurseMission(Node):
    def __init__(self):
        super().__init__('mission_6_nurse')
        
        self.create_subscription(PoseStamped, '/go1_pose', self.pose_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        
        self.path_pub = self.create_publisher(Path, '/local_path', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # 직접 제어용
        
        self.planner = AStarPlanner()
        self.current_pose = None
        self.map_received = False
        self.is_circling = False
        self.circle_start_time = 0
        
        self.timer = self.create_timer(0.2, self.control_loop) # 주기를 좀 빠르게(0.2s)
        self.get_logger().info("Mission 6: Find Nurse and Rotate")

    def pose_cb(self, msg):
        self.current_pose = msg

    def map_cb(self, msg):
        self.planner.update_map(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin)
        self.map_received = True

    def control_loop(self):
        if self.current_pose is None: return

        # 1. 빙글빙글 도는 모드인지 확인
        if self.is_circling:
            # 10초 동안 돌기
            if time.time() - self.circle_start_time > 10.0:
                self.get_logger().info("Done Circling!")
                self.vel_pub.publish(Twist()) # 정지
                raise SystemExit
            
            # 원 그리기 명령 (선속도 + 각속도 동시에 주면 원형 이동)
            twist = Twist()
            twist.linear.x = 0.3  # 앞으로
            twist.angular.z = 0.5 # 왼쪽으로 회전
            self.vel_pub.publish(twist)
            return

        # 2. 아직 도착 안 했으면 -> A*로 접근
        if not self.map_received: return

        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y
        dist = math.sqrt((NURSE_X - curr_x)**2 + (NURSE_Y - curr_y)**2)

        if dist < APPROACH_DIST:
            self.get_logger().info("Met Nurse! Start Circling...")
            self.is_circling = True
            self.circle_start_time = time.time()
            # Path Tracker가 방해하지 않게 빈 경로 발행
            self.path_pub.publish(Path(header=self.current_pose.header))
            return

        # 3. 경로 생성 및 이동
        path_list = self.planner.plan((curr_x, curr_y), (NURSE_X, NURSE_Y))
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
        rclpy.spin(NurseMission())
    except SystemExit:
        rclpy.shutdown()

if __name__ == '__main__':
    main()