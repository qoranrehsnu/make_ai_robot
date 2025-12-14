#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, Imu
from tf2_ros import TransformBroadcaster
import tf_transformations
import numpy as np

from utils import scan_to_pcd, icp_2d_point_to_line_with_cov, wrap_angle


def rot2(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


class OdomLocalizer(Node):
    """
    Odom localizer (map 1:1 추종용, 최종 안정판)

    개선 사항:
      1) 저속/정지 구간 미세 translation 노이즈 컷
      2) 회전 중(scan ICP 오차) translation 억제

    ❗ EMA, scale, hard stationary gate 없음 (1:1 유지)
    """

    def __init__(self):
        super().__init__("odom_localizer_node")

        # Topics / frames
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base")

        # ICP
        self.declare_parameter("scan_stride", 2)
        self.declare_parameter("icp_max_iter", 8)
        self.declare_parameter("icp_dist_thresh", 0.40)
        self.declare_parameter("icp_min_corr", 25)
        self.declare_parameter("icp_huber", 0.25)
        self.declare_parameter("icp_damping", 1e-4)
        self.declare_parameter("icp_cond_max", 1e9)

        # yaw fusion
        self.declare_parameter("use_icp_yaw", True)
        self.declare_parameter("icp_yaw_gain", 1.0)
        self.declare_parameter("icp_yaw_max_abs", np.deg2rad(10.0))

        # IMU bias calib
        self.declare_parameter("bias_calib_seconds", 3.0)
        self.declare_parameter("bias_calib_gyro_abs_max", np.deg2rad(2.0))

        # Logging
        self.declare_parameter("log_every_n_scans", 25)

        # Load params
        self.imu_topic = self.get_parameter("imu_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.scan_stride = int(self.get_parameter("scan_stride").value)
        self.icp_max_iter = int(self.get_parameter("icp_max_iter").value)
        self.icp_dist_thresh = float(self.get_parameter("icp_dist_thresh").value)
        self.icp_min_corr = int(self.get_parameter("icp_min_corr").value)
        self.icp_huber = float(self.get_parameter("icp_huber").value)
        self.icp_damping = float(self.get_parameter("icp_damping").value)
        self.icp_cond_max = float(self.get_parameter("icp_cond_max").value)

        self.use_icp_yaw = bool(self.get_parameter("use_icp_yaw").value)
        self.icp_yaw_gain = float(self.get_parameter("icp_yaw_gain").value)
        self.icp_yaw_max_abs = float(self.get_parameter("icp_yaw_max_abs").value)

        self.bias_calib_seconds = float(self.get_parameter("bias_calib_seconds").value)
        self.bias_calib_gyro_abs_max = float(self.get_parameter("bias_calib_gyro_abs_max").value)

        self.log_every_n_scans = int(self.get_parameter("log_every_n_scans").value)

        # ROS
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.tf_pub = TransformBroadcaster(self)

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # IMU
        self.bg = 0.0
        self.last_imu_time = None
        self.latest_wz = 0.0

        # Scan
        self.prev_scan_pcd = None
        self.scan_count = 0

        # IMU bias calibration
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.calib_sum = 0.0
        self.calib_cnt = 0

        self.get_logger().info("[ODOM] 1:1 mode + 개선1,2 적용")

    def imu_cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9

        wz = float(msg.angular_velocity.z)
        self.latest_wz = wz

        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self.start_time) < self.bias_calib_seconds and abs(wz) < self.bias_calib_gyro_abs_max:
            self.calib_sum += wz
            self.calib_cnt += 1
            if self.calib_cnt >= 50:
                self.bg = self.calib_sum / self.calib_cnt

        if self.last_imu_time is None:
            self.last_imu_time = t
            return

        dt = t - self.last_imu_time
        self.last_imu_time = t
        if dt <= 0.0 or dt > 0.5:
            return

        self.yaw = wrap_angle(self.yaw + (wz - self.bg) * dt)

    def scan_cb(self, scan: LaserScan):
        self.scan_count += 1

        pcd = scan_to_pcd(scan, stride=self.scan_stride)
        if pcd.shape[0] < 50:
            self.publish_tf(scan.header.stamp)
            return

        if self.prev_scan_pcd is None:
            self.prev_scan_pcd = pcd
            self.publish_tf(scan.header.stamp)
            return

        T, Cov, info = icp_2d_point_to_line_with_cov(
            src=self.prev_scan_pcd,
            tgt=pcd,
            max_iter=self.icp_max_iter,
            dist_thresh=self.icp_dist_thresh,
            min_corr=self.icp_min_corr,
            huber_delta=self.icp_huber,
            damping=self.icp_damping,
        )

        T = np.linalg.inv(T)
        num_corr = int(info.get("num_corr", 0))
        cond = info.get("cond", None)

        if num_corr < self.icp_min_corr or (cond is not None and cond > self.icp_cond_max):
            self.prev_scan_pcd = pcd
            self.publish_tf(scan.header.stamp)
            return

        dx_b = float(T[0, 2])
        dy_b = float(T[1, 2])
        dth_icp = float(np.arctan2(T[1, 0], T[0, 0]))

        # === 개선 1: 저속 미세 노이즈 컷 ===
        if abs(dx_b) < 1e-4:
            dx_b = 0.0
        if abs(dy_b) < 1e-4:
            dy_b = 0.0

        # === 개선 2: 회전 중 translation 억제 ===
        if abs(self.latest_wz - self.bg) > 0.6:  # rad/s
            dx_b = 0.0
            dy_b = 0.0

        # 1:1 누적
        dp_w = rot2(self.yaw) @ np.array([dx_b, dy_b], dtype=np.float64)
        self.x += float(dp_w[0])
        self.y += float(dp_w[1])

        if self.use_icp_yaw and abs(dth_icp) < self.icp_yaw_max_abs:
            self.yaw = wrap_angle(self.yaw + self.icp_yaw_gain * dth_icp)

        self.prev_scan_pcd = pcd

        if (self.scan_count % self.log_every_n_scans) == 0:
            self.get_logger().info(
                f"[ODOM] x={self.x:.3f} y={self.y:.3f} yaw={np.rad2deg(self.yaw):.1f}deg"
            )

        self.publish_tf(scan.header.stamp)

    def publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, float(self.yaw))
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        self.tf_pub.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

