#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped, Twist
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
    Odom localizer (1:1 mode)

    개선 반영:
      (2) ICP yaw 과신 방지 (gain ↓ + 회전 중 비활성)
      (3) 회전 중 translation 완전 차단 → 감쇠
    """

    def __init__(self):
        super().__init__("odom_localizer_node")

        # Topics / frames
        self.declare_parameter("imu_topic", "/imu_plugin/out")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base")

        # ICP
        self.declare_parameter("scan_stride", 2)
        self.declare_parameter("icp_max_iter", 8)
        self.declare_parameter("icp_dist_thresh", 0.40)
        self.declare_parameter("icp_min_corr", 25)
        self.declare_parameter("icp_huber", 0.25)
        self.declare_parameter("icp_damping", 1e-4)

        # ICP yaw
        self.declare_parameter("use_icp_yaw", True)
        self.declare_parameter("icp_yaw_gain", 0.5)          # 감소
        self.declare_parameter("icp_yaw_max_abs", np.deg2rad(10.0))
        self.declare_parameter("yaw_gate_rate", 0.6)        # rad/s

        # 회전 중 translation 감쇠
        self.declare_parameter("rot_trans_scale", 0.2)      # 0이면 기존 방식

        # IMU yaw fusion
        self.declare_parameter("use_imu_orientation_yaw", True)
        self.declare_parameter("imu_yaw_gain", 0.2)

        # gyro bias
        self.declare_parameter("bias_calib_seconds", 3.0)
        self.declare_parameter("bias_calib_gyro_abs_max", np.deg2rad(2.0))
        self.declare_parameter("bias_update_alpha", 0.01)

        # stationary 판단
        self.declare_parameter("stationary_lin_max", 0.03)
        self.declare_parameter("stationary_ang_max", 0.06)
        self.declare_parameter("stationary_gyro_max", np.deg2rad(3.0))

        # logging
        self.declare_parameter("log_every_n_scans", 25)

        # Load params
        self.imu_topic = self.get_parameter("imu_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.use_icp_yaw = self.get_parameter("use_icp_yaw").value
        self.icp_yaw_gain = self.get_parameter("icp_yaw_gain").value
        self.icp_yaw_max_abs = self.get_parameter("icp_yaw_max_abs").value
        self.yaw_gate_rate = self.get_parameter("yaw_gate_rate").value

        self.rot_trans_scale = self.get_parameter("rot_trans_scale").value

        self.use_imu_orientation_yaw = self.get_parameter("use_imu_orientation_yaw").value
        self.imu_yaw_gain = self.get_parameter("imu_yaw_gain").value

        self.bias_calib_seconds = self.get_parameter("bias_calib_seconds").value
        self.bias_calib_gyro_abs_max = self.get_parameter("bias_calib_gyro_abs_max").value
        self.bias_update_alpha = self.get_parameter("bias_update_alpha").value

        self.stationary_lin_max = self.get_parameter("stationary_lin_max").value
        self.stationary_ang_max = self.get_parameter("stationary_ang_max").value
        self.stationary_gyro_max = self.get_parameter("stationary_gyro_max").value

        self.log_every_n_scans = self.get_parameter("log_every_n_scans").value

        # ROS
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_cb, 10)
        self.tf_pub = TransformBroadcaster(self)

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # IMU
        self.bg = 0.0
        self.last_imu_time = None
        self.latest_wz = 0.0
        self.latest_cmd_v = 0.0
        self.latest_cmd_w = 0.0

        # Scan
        self.prev_scan_pcd = None
        self.scan_count = 0

        # bias init
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.calib_sum = 0.0
        self.calib_cnt = 0

        self.get_logger().info("[ODOM] ICP yaw softened + rotation translation scaled")

    def cmd_vel_cb(self, msg: Twist):
        self.latest_cmd_v = msg.linear.x
        self.latest_cmd_w = msg.angular.z

    def imu_cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9

        wz = msg.angular_velocity.z
        self.latest_wz = wz
        now = self.get_clock().now().nanoseconds * 1e-9

        if (now - self.start_time) < self.bias_calib_seconds and abs(wz) < self.bias_calib_gyro_abs_max:
            self.calib_sum += wz
            self.calib_cnt += 1
            if self.calib_cnt > 50:
                self.bg = self.calib_sum / self.calib_cnt

        if self.last_imu_time is None:
            self.last_imu_time = t
            return

        dt = t - self.last_imu_time
        self.last_imu_time = t
        if dt <= 0.0 or dt > 0.5:
            return

        yaw_pred = wrap_angle(self.yaw + (wz - self.bg) * dt)

        if self.use_imu_orientation_yaw:
            q = msg.orientation
            n = np.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if n > 1e-6:
                _, _, yaw_meas = tf_transformations.euler_from_quaternion(
                    [q.x/n, q.y/n, q.z/n, q.w/n]
                )
                err = wrap_angle(yaw_meas - yaw_pred)
                self.yaw = wrap_angle(yaw_pred + self.imu_yaw_gain * err)
            else:
                self.yaw = yaw_pred
        else:
            self.yaw = yaw_pred

        if (abs(self.latest_cmd_v) < self.stationary_lin_max and
            abs(self.latest_cmd_w) < self.stationary_ang_max and
            abs(wz) < self.stationary_gyro_max):
            a = self.bias_update_alpha
            self.bg = (1.0 - a) * self.bg + a * wz

    def scan_cb(self, scan: LaserScan):
        self.scan_count += 1

        pcd = scan_to_pcd(scan)
        if pcd.shape[0] < 50:
            self.publish_tf(scan.header.stamp)
            return

        if self.prev_scan_pcd is None:
            self.prev_scan_pcd = pcd
            self.publish_tf(scan.header.stamp)
            return

        T, _, info = icp_2d_point_to_line_with_cov(self.prev_scan_pcd, pcd)
        T = np.linalg.inv(T)

        if info.get("num_corr", 0) < 25:
            self.prev_scan_pcd = pcd
            self.publish_tf(scan.header.stamp)
            return

        dx_b = T[0, 2]
        dy_b = T[1, 2]
        dth = np.arctan2(T[1, 0], T[0, 0])

        # 🔧 회전 중 translation 감쇠
        if abs(self.latest_wz - self.bg) > self.yaw_gate_rate:
            dx_b *= self.rot_trans_scale
            dy_b *= self.rot_trans_scale

        dp = rot2(self.yaw) @ np.array([dx_b, dy_b])
        self.x += dp[0]
        self.y += dp[1]

        # 🔧 ICP yaw: 직진 중 + gain ↓
        if (self.use_icp_yaw and
            abs(dth) < self.icp_yaw_max_abs and
            abs(self.latest_wz - self.bg) < self.yaw_gate_rate):
            self.yaw = wrap_angle(self.yaw + self.icp_yaw_gain * dth)

        self.prev_scan_pcd = pcd

        if self.scan_count % self.log_every_n_scans == 0:
            self.get_logger().info(
                f"[ODOM] x={self.x:.3f} y={self.y:.3f} yaw={np.rad2deg(self.yaw):.1f}deg bg={self.bg:.5f}"
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

        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

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

