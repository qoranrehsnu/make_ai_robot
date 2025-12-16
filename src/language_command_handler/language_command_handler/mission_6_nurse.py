#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String, Float32
import math
import time

# [의존성] 사용자의 AStarPlanner 모듈 임포트
# 만약 경로 에러가 난다면 이 파일이 같은 패키지 내에 있는지 확인해주세요.
from language_command_handler.astar_planner import AStarPlanner

# ==========================================
# ★★★ 대략적인 간호사 방 위치 ★★★
# ==========================================
NURSE_ROOM_X = -8.5
NURSE_ROOM_Y = -27.25

# ==========================================
# [MOD] 미션 파라미터
# ==========================================
APPROACH_DIST = 2.0          # 방 근처 진입 판단 거리 (미터)
TARGET_DIST = 1.2            # 간호사 앞 정지 거리 (미터)
CENTER_TOL = 0.12            # (Perception x좌표 부재로 미사용)
MAX_ANG = 0.8                # 최대 회전 속도 (rad/s)
MAX_LIN = 0.35               # 최대 전진 속도 (m/s)

# [MOD] 회전(한 바퀴) 설정
CIRCLE_RADIUS = 1.2          # 목표 회전 반경 (m)
CIRCLE_ANG_Z = 0.55          # 목표 각속도 (rad/s) - 단, MAX_LIN에 의해 자동 조정됨

class NurseMission(Node):
    def __init__(self):
        super().__init__('mission_6_nurse')

        # ---------------------------------------------------
        # Subscribers
        # ---------------------------------------------------
        self.create_subscription(PoseStamped, '/go1_pose', self.pose_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        # Perception Nodes
        self.create_subscription(String, '/detections/labels', self.labels_cb, 10)
        self.create_subscription(Float32, '/detections/distance', self.dist_cb, 10)

        # ---------------------------------------------------
        # Publishers
        # ---------------------------------------------------
        self.path_pub = self.create_publisher(Path, '/local_path', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---------------------------------------------------
        # Internal State
        # ---------------------------------------------------
        self.planner = AStarPlanner()
        self.current_pose = None
        self.map_received = False

        # State Machine: GO_ROOM -> FIND_NURSE -> APPROACH_FACE -> CIRCLE -> DONE
        self.state = "GO_ROOM"    

        # Perception Variables
        self.nurse_seen = False
        self.nurse_distance = None
       
        # Timer variables
        self.circle_start_time = None
       
        # Control Loop (0.2s = 5Hz)
        self.timer = self.create_timer(0.2, self.control_loop)

        self.get_logger().info("Mission 6 Started: Go Room -> Find -> Approach -> Circle")

    # -------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------
    def pose_cb(self, msg):
        self.current_pose = msg

    def map_cb(self, msg):
        # AStarPlanner 업데이트
        self.planner.update_map(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin)
        self.map_received = True

    def labels_cb(self, msg: String):
        # 'nurse' 문자열 포함 여부 확인
        s = msg.data.lower()
        self.nurse_seen = ("nurse" in s)

    def dist_cb(self, msg: Float32):
        d = float(msg.data)
        # 유효한 거리 값만 업데이트
        if d > 0.0 and not math.isinf(d) and not math.isnan(d):
            self.nurse_distance = d

    # -------------------------------------------------------
    # Helper Functions
    # -------------------------------------------------------
    def publish_stop(self):
        """로봇 정지 및 경로 초기화"""
        self.vel_pub.publish(Twist())
       
        p = Path()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(p)

    def publish_path_to(self, gx, gy):
        """A* 경로 생성 및 발행"""
        if not self.map_received or self.current_pose is None:
            return

        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y

        # A* Planner 호출
        path_list = self.planner.plan((curr_x, curr_y), (gx, gy))
       
        if not path_list:
            # 경로 생성 실패 시 로그 (너무 자주는 아니게)
            # self.get_logger().debug("Planner failed or empty path")
            return

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for (wx, wy) in path_list:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    # -------------------------------------------------------
    # Main Control Loop
    # -------------------------------------------------------
    def control_loop(self):
        if self.current_pose is None:
            return

        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y

        # =========================================================
        # 1. GO_ROOM: 방 좌표 근처까지 이동
        # =========================================================
        if self.state == "GO_ROOM":
            if not self.map_received:
                self.get_logger().info("Waiting for map...")
                return

            dx = NURSE_ROOM_X - curr_x
            dy = NURSE_ROOM_Y - curr_y
            dist_to_room = math.hypot(dx, dy)

            # 방 근처에 도착했는지 확인
            if dist_to_room > APPROACH_DIST:
                self.publish_path_to(NURSE_ROOM_X, NURSE_ROOM_Y)
            else:
                self.get_logger().info(f"Arrived near nurse room ({dist_to_room:.2f}m). Mode: FIND_NURSE")
                self.publish_stop()
                self.state = "FIND_NURSE"

        # =========================================================
        # 2. FIND_NURSE: 제자리 회전하며 탐색
        # =========================================================
        elif self.state == "FIND_NURSE":
            if self.nurse_seen:
                self.get_logger().info("Nurse detected! Mode: APPROACH_FACE")
                self.publish_stop()
                self.state = "APPROACH_FACE"
            else:
                # 못 찾았으면 제자리 회전 (속도 제한 준수)
                twist = Twist()
                twist.angular.z = min(0.4, MAX_ANG)
                self.vel_pub.publish(twist)

        # =========================================================
        # 3. APPROACH_FACE: 간호사 정면 일정 거리까지 접근
        # =========================================================
        elif self.state == "APPROACH_FACE":
            # 트래킹 손실 시 다시 탐색 모드로 복귀
            if not self.nurse_seen:
                self.get_logger().warn("Lost nurse track. Searching again...")
                self.publish_stop()
                self.state = "FIND_NURSE"
                return

            # 거리 데이터가 아직 없으면 대기
            if self.nurse_distance is None:
                self.publish_stop()
                return

            # 목표 거리 도달 확인
            if self.nurse_distance <= TARGET_DIST:
                self.get_logger().info(f"Reached target distance ({self.nurse_distance:.2f}m). Mode: CIRCLE")
                self.publish_stop()
                self.state = "CIRCLE"
                self.circle_start_time = None # 타이머 초기화 플래그
            else:
                # 접근 (직진)
                twist = Twist()
                approach_speed = 0.2
                twist.linear.x = min(approach_speed, MAX_LIN)
                # (중앙 정렬 로직이 필요하다면 여기에 angular.z 추가)
                self.vel_pub.publish(twist)

        # =========================================================
        # 4. CIRCLE: 간호사를 중심으로 한 바퀴 돌기 (물리 보정 적용)
        # =========================================================
        elif self.state == "CIRCLE":
            # 타이머 시작 설정
            if self.circle_start_time is None:
                self.circle_start_time = self.get_clock().now().nanoseconds / 1e9
                return

            # [핵심 수정] 물리적으로 가능한 속도와 시간 계산
            # 1. 목표로 하는 선속도 (v = r * w)
            req_linear_v = CIRCLE_RADIUS * CIRCLE_ANG_Z
           
            # 2. 로봇의 물리적 한계(MAX_LIN)에 맞춰 속도 클램핑
            # (요구 속도가 0.66인데 MAX가 0.35라면 0.35로 제한됨)
            actual_linear_v = min(req_linear_v, MAX_LIN)
           
            # 3. 실제 선속도에 맞춰 각속도(w = v / r) 재계산
            # 이렇게 해야 원의 반지름(1.2m)이 유지됨
            actual_angular_z = actual_linear_v / CIRCLE_RADIUS
           
            # 4. 실제 각속도에 맞춰 "한 바퀴(2pi) 도는 데 걸리는 시간" 재계산
            needed_time = (2.0 * math.pi) / actual_angular_z

            # 경과 시간 확인
            now = self.get_clock().now().nanoseconds / 1e9
            elapsed = now - self.circle_start_time

            if elapsed < needed_time:
                twist = Twist()
                twist.linear.x = actual_linear_v
                twist.angular.z = actual_angular_z
                self.vel_pub.publish(twist)
               
                # 디버깅 로그 (필요시 주석 해제)
                # if elapsed % 1.0 < 0.2:
                #    self.get_logger().info(f"Circling... T:{elapsed:.1f}/{needed_time:.1f}")
            else:
                self.get_logger().info("Circle mission finished. Mode: DONE")
                self.publish_stop()
                self.state = "DONE"

        # =========================================================
        # 5. DONE: 종료
        # =========================================================
        elif self.state == "DONE":
            pass

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