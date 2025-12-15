#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

import numpy as np
import tf_transformations

from utils import map_to_pcd, build_kdtree, wrap_angle


class GlobalLocalizerPF1to1(Node):
    """
    Particle Filter for map->odom correction (1:1 follow mode).
    Publishes TF: map -> odom

    목표:
      - map이 "덜 움직이는" 현상 제거
      - TF low-pass, stride 감쇠, yaw freeze 제거
      - PF 추정치로 계산된 map->odom을 그대로 출력 (즉시 반영)
    """

    def __init__(self):
        super().__init__("global_localizer_node")

        # Topics / frames
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base")  # 네 환경 기준

        # PF params
        self.declare_parameter("num_particles", 350)
        self.declare_parameter("init_x", 0.0)
        self.declare_parameter("init_y", 1.0)
        self.declare_parameter("init_yaw", 0.0)
        self.declare_parameter("init_std_xy", 0.6)
        self.declare_parameter("init_std_yaw_deg", 25.0)

        self.declare_parameter("odom_noise_xy", 0.02)
        self.declare_parameter("odom_noise_yaw_deg", 1.5)

        # Measurement
        self.declare_parameter("scan_step", 15)
        self.declare_parameter("max_scan_points", 70)
        self.declare_parameter("map_sigma", 0.45)
        self.declare_parameter("resample_thresh", 0.6)

        # 1:1 follow output
        self.declare_parameter("pf_update_stride", 1)  # 반드시 1
        self.declare_parameter("tf_alpha", 1.0)        # 반드시 1.0 (LPF off)
        self.declare_parameter("freeze_yaw", False)
        self.declare_parameter("yaw_alpha", 1.0)       # 즉시 반영

        # publish
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("log_every_n_scans", 30)

        # Load params
        self.map_topic = self.get_parameter("map_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.N = int(self.get_parameter("num_particles").value)

        self.init_x = float(self.get_parameter("init_x").value)
        self.init_y = float(self.get_parameter("init_y").value)
        self.init_yaw = float(self.get_parameter("init_yaw").value)

        init_std_xy = float(self.get_parameter("init_std_xy").value)
        init_std_yaw = np.deg2rad(float(self.get_parameter("init_std_yaw_deg").value))

        self.odom_noise = np.array([
            float(self.get_parameter("odom_noise_xy").value),
            float(self.get_parameter("odom_noise_xy").value),
            np.deg2rad(float(self.get_parameter("odom_noise_yaw_deg").value)),
        ], dtype=np.float64)

        self.scan_step = int(self.get_parameter("scan_step").value)
        self.max_scan_points = int(self.get_parameter("max_scan_points").value)
        self.map_sigma = float(self.get_parameter("map_sigma").value)
        self.resample_thresh = float(self.get_parameter("resample_thresh").value)

        self.pf_update_stride = int(self.get_parameter("pf_update_stride").value)
        self.tf_alpha = float(self.get_parameter("tf_alpha").value)
        self.freeze_yaw = bool(self.get_parameter("freeze_yaw").value)
        self.yaw_alpha = float(self.get_parameter("yaw_alpha").value)

        pub_hz = float(self.get_parameter("publish_rate_hz").value)
        self.log_every_n_scans = int(self.get_parameter("log_every_n_scans").value)

        # ROS
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, 1)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0 / pub_hz, self.publish_tf_timer)

        # PF state
        self.p = np.zeros((self.N, 3), dtype=np.float64)
        self.w = np.ones(self.N, dtype=np.float64) / self.N

        self.p[:, 0] = self.init_x + np.random.normal(0.0, init_std_xy, self.N)
        self.p[:, 1] = self.init_y + np.random.normal(0.0, init_std_xy, self.N)
        self.p[:, 2] = wrap_angle(self.init_yaw + np.random.normal(0.0, init_std_yaw, self.N))

        self.map_pcd = None
        self.map_tree = None

        self.last_odom = None

        # map->odom TF state (start from init)
        self.last_tf_map_to_odom = np.array([self.init_x, self.init_y, self.init_yaw], dtype=np.float64)

        self.scan_count = 0

        self.get_logger().info("[GLOBAL 1:1] started (tf_alpha=1.0, stride=1, yaw unfrozen)")

    def map_cb(self, msg: OccupancyGrid):
        self.map_pcd = map_to_pcd(msg, threshold=50)
        self.map_tree = build_kdtree(self.map_pcd)
        self.get_logger().info(f"[MAP] loaded pcd={len(self.map_pcd)} tree={'YES' if self.map_tree else 'NO'}")

    def get_odom_pose(self):
        # odom -> base
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return np.array([tx, ty, yaw], dtype=np.float64)

    def motion_update(self, odom_now, odom_prev):
        dx = odom_now[0] - odom_prev[0]
        dy = odom_now[1] - odom_prev[1]
        dyaw = wrap_angle(odom_now[2] - odom_prev[2])

        cy = np.cos(odom_prev[2])
        sy = np.sin(odom_prev[2])
        local_dx = cy * dx + sy * dy
        local_dy = -sy * dx + cy * dy

        cp = np.cos(self.p[:, 2])
        sp = np.sin(self.p[:, 2])

        self.p[:, 0] += cp * local_dx - sp * local_dy + np.random.normal(0, self.odom_noise[0], self.N)
        self.p[:, 1] += sp * local_dx + cp * local_dy + np.random.normal(0, self.odom_noise[1], self.N)
        self.p[:, 2] = wrap_angle(self.p[:, 2] + dyaw + np.random.normal(0, self.odom_noise[2], self.N))

    def measurement_update(self, scan: LaserScan):
        if self.map_tree is None or self.map_pcd is None:
            return

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        idx = np.where(
            (ranges > scan.range_min) &
            (ranges < scan.range_max) &
            np.isfinite(ranges)
        )[0]
        if idx.size == 0:
            return

        idx = idx[::self.scan_step]
        if idx.size > self.max_scan_points:
            idx = idx[:self.max_scan_points]

        r = ranges[idx]
        ang = scan.angle_min + idx * scan.angle_increment
        sx = r * np.cos(ang)
        sy = r * np.sin(ang)

        cp = np.cos(self.p[:, 2])[:, None]
        sp = np.sin(self.p[:, 2])[:, None]

        wx = self.p[:, 0][:, None] + sx[None, :] * cp - sy[None, :] * sp
        wy = self.p[:, 1][:, None] + sx[None, :] * sp + sy[None, :] * cp

        pts = np.column_stack((wx.reshape(-1), wy.reshape(-1)))
        dists, _ = self.map_tree.query(pts, k=1)
        dists = dists.reshape(self.N, -1)

        inv2sig2 = 1.0 / (2.0 * (self.map_sigma ** 2))
        ll = -np.mean((dists ** 2) * inv2sig2, axis=1)   # log-likelihood
        ll -= np.max(ll)

        w = np.exp(ll) + 1e-12
        w /= np.sum(w)
        self.w = w

    def resample(self):
        neff = 1.0 / np.sum(self.w ** 2)
        if neff >= self.resample_thresh * self.N:
            return

        positions = (np.arange(self.N) + np.random.rand()) / self.N
        idx = np.zeros(self.N, dtype=np.int32)
        cumsum = np.cumsum(self.w)

        i = j = 0
        while i < self.N:
            if positions[i] < cumsum[j]:
                idx[i] = j
                i += 1
            else:
                j += 1

        self.p = self.p[idx]
        self.w[:] = 1.0 / self.N

    def estimate_pose(self):
        mx = np.sum(self.p[:, 0] * self.w)
        my = np.sum(self.p[:, 1] * self.w)
        cy = np.sum(np.cos(self.p[:, 2]) * self.w)
        sy = np.sum(np.sin(self.p[:, 2]) * self.w)
        myaw = np.arctan2(sy, cy)
        return np.array([mx, my, myaw], dtype=np.float64)

    def scan_cb(self, scan: LaserScan):
        if self.map_tree is None:
            return

        self.scan_count += 1

        odom = self.get_odom_pose()
        if odom is None:
            return

        # motion update always
        if self.last_odom is not None:
            self.motion_update(odom, self.last_odom)
        self.last_odom = odom

        # PF update stride
        if (self.scan_count % self.pf_update_stride) != 0:
            return

        self.measurement_update(scan)
        self.resample()

        est = self.estimate_pose()

        # map->odom = map->base(est) * inv(odom->base)
        ox, oy, oyaw = odom
        ex, ey, eyaw = est

        c = np.cos(eyaw)
        s = np.sin(eyaw)

        tx = ex - (c * ox - s * oy)
        ty = ey - (s * ox + c * oy)
        raw_tyaw = wrap_angle(eyaw - oyaw)

        # 1:1 output (NO low-pass)
        self.last_tf_map_to_odom[0] = tx
        self.last_tf_map_to_odom[1] = ty
        self.last_tf_map_to_odom[2] = raw_tyaw

        if (self.scan_count % self.log_every_n_scans) == 0:
            neff = 1.0 / np.sum(self.w ** 2)
            self.get_logger().info(
                f"[GLOBAL 1:1] map->odom x={tx:.2f} y={ty:.2f} yaw={np.rad2deg(raw_tyaw):.1f}deg neff={neff:.1f}"
            )

    def publish_tf_timer(self):
        tx, ty, tyaw = self.last_tf_map_to_odom

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame

        t.transform.translation.x = float(tx)
        t.transform.translation.y = float(ty)
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, float(tyaw))
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizerPF1to1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()