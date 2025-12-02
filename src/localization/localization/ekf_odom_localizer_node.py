#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def scan_to_points(scan: LaserScan,
                   min_range: float = 0.1,
                   max_range: float | None = None,
                   step: int = 2) -> np.ndarray:
    angles = scan.angle_min + np.arange(0, len(scan.ranges), step) * scan.angle_increment
    ranges = np.array(scan.ranges[::step], dtype=np.float32)

    if max_range is None:
        max_range = scan.range_max

    valid = np.logical_and(ranges > min_range, ranges < max_range)
    angles = angles[valid]
    ranges = ranges[valid]

    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)
    return np.stack((xs, ys), axis=1)


def best_fit_transform_2d(src: np.ndarray, dst: np.ndarray):
    assert src.shape == dst.shape
    src_centroid = src.mean(axis=0)
    dst_centroid = dst.mean(axis=0)

    src_centered = src - src_centroid
    dst_centered = dst - dst_centroid

    H = src_centered.T @ dst_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T

    t = dst_centroid - R @ src_centroid
    return R, t


def icp_2d(previous: np.ndarray,
           current: np.ndarray,
           max_iters: int = 30,
           tol: float = 1e-4):
    if previous.shape[0] < 10 or current.shape[0] < 10:
        return np.eye(3)

    dst = previous
    src = current
    T = np.eye(3)

    for _ in range(max_iters):
        src_h = np.c_[src, np.ones(src.shape[0])]
        src_tf = (T @ src_h.T).T[:, :2]

        dists = np.linalg.norm(src_tf[:, None, :] - dst[None, :, :], axis=2)
        idx = np.argmin(dists, axis=1)
        matched_dst = dst[idx]

        R, t = best_fit_transform_2d(src_tf, matched_dst)
        dT = np.eye(3)
        dT[:2, :2] = R
        dT[:2, 2] = t

        T_new = dT @ T

        trans_delta = np.linalg.norm(T_new[:2, 2] - T[:2, 2])
        rot_delta = math.acos(
            max(-1.0, min(1.0, (T_new[0, 0] + T_new[1, 1]) / 2.0))
        )
        T = T_new
        if trans_delta < tol and rot_delta < tol:
            break

    return T


class EkfOdomLocalizer(Node):
    """
    IMU + ICP EKF 기반 odom -> base 추정 노드
    상태: [x, y, yaw, vx, vy]^T (2D 평면)
    - 예측: IMU
    - 보정: ICP (스캔 매칭)
    """

    def __init__(self):
        super().__init__('ekf_odom_localizer')

        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu/data')

        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value

        # EKF 상태
        # x = [x, y, yaw, vx, vy]^T
        self.x = np.zeros((5, 1))
        self.P = np.eye(5) * 1e-3

        # 프로세스/측정 잡음 (튜닝 필요)
        self.Q = np.diag([1e-4, 1e-4, 1e-5, 1e-2, 1e-2])
        self.R_icp = np.diag([1e-3, 1e-3, 1e-4])  # ICP pose noise

        self.last_imu_time = None

        # ICP용 상태
        self.prev_points = None

        # 센서 최신값
        self.last_imu_msg: Imu | None = None

        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub_scan = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )
        self.sub_imu = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            100
        )

        self.get_logger().info(
            f'ekf_odom_localizer started. odom={self.odom_frame}, base={self.base_frame}, '
            f'scan={self.scan_topic}, imu={self.imu_topic}'
        )

    # ---------- IMU EKF prediction ----------

    def imu_callback(self, msg: Imu):
        # dt 계산
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu_time is None:
            self.last_imu_time = t
            self.last_imu_msg = msg
            return
        dt = t - self.last_imu_time
        if dt <= 0.0 or dt > 0.2:
            dt = 0.0
        self.last_imu_time = t
        self.last_imu_msg = msg

        if dt <= 0.0:
            return

        # 현재 상태
        x, y, yaw, vx, vy = self.x.flatten()

        # IMU에서 각속도/가속도 읽기 (base frame 기준)
        wz = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y

        # yaw 예측
        yaw_pred = yaw + wz * dt

        # 가속도를 world frame으로 회전
        c = math.cos(yaw)
        s = math.sin(yaw)
        ax_w = c * ax - s * ay
        ay_w = s * ax + c * ay

        # 속도 예측
        vx_pred = vx + ax_w * dt
        vy_pred = vy + ay_w * dt

        # 위치 예측
        x_pred = x + vx_pred * dt
        y_pred = y + vy_pred * dt

        # 상태 예측
        self.x = np.array([[x_pred], [y_pred], [yaw_pred], [vx_pred], [vy_pred]])

        # 선형화된 상태 전이 Jacobian F
        F = np.eye(5)
        # yaw에 대한 속도/위치 의존성은 단순화 (작은 dt 가정)
        F[0, 3] = dt   # dx/dvx
        F[1, 4] = dt   # dy/dvy
        F[2, 2] = 1.0  # yaw

        # 공분산 예측
        self.P = F @ self.P @ F.T + self.Q

        # TF publish (IMU만으로도 계속 pose 내보냄)
        self.publish_tf(msg.header.stamp)

    # ---------- ICP 측정 (LaserScan) ----------

    def scan_callback(self, msg: LaserScan):
        points = scan_to_points(msg, step=2)

        if self.prev_points is None:
            self.prev_points = points
            return

        T = icp_2d(self.prev_points, points)

        dx = T[0, 2]
        dy = T[1, 2]
        dyaw = math.atan2(T[1, 0], T[0, 0])

        # 현재 EKF 상태에서 ICP 결과를 "절대 pose 측정"으로 변환 (누적)
        # 여기서는 간단히: z = [x + dx_body, y + dy_body, yaw + dyaw]
        x, y, yaw, vx, vy = self.x.flatten()

        c = math.cos(yaw)
        s = math.sin(yaw)
        # body frame에서 나온 dx,dy를 world frame으로
        dx_w = c * dx - s * dy
        dy_w = s * dx + c * dy

        z_x = x + dx_w
        z_y = y + dy_w
        z_yaw = yaw + dyaw

        z = np.array([[z_x], [z_y], [z_yaw]])

        # 측정 모델: h(x) = [x, y, yaw]^T
        H = np.zeros((3, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0

        z_hat = H @ self.x
        y_res = z - z_hat  # innovation

        S = H @ self.P @ H.T + self.R_icp
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y_res
        I = np.eye(5)
        self.P = (I - K @ H) @ self.P

        # ICP 기준 스캔 저장
        self.prev_points = points

        # TF publish (ICP 보정 후)
        self.publish_tf(msg.header.stamp)

    # ---------- TF publish ----------

    def publish_tf(self, stamp):
        x, y, yaw, vx, vy = self.x.flatten()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EkfOdomLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

