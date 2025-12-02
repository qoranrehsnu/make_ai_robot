#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# AMCL likelihood field용 distance transform 계산
from scipy.ndimage import distance_transform_edt


# ---------------------------------------------------------
# LaserScan → (angles, ranges) 다운샘플 함수
# ---------------------------------------------------------
def scan_to_angles_ranges(scan: LaserScan, step: int = 8):
    angles = scan.angle_min + np.arange(0, len(scan.ranges), step) * scan.angle_increment
    ranges = np.array(scan.ranges[::step], dtype=np.float32)

    valid = np.logical_and(ranges > scan.range_min, ranges < scan.range_max)
    return angles[valid], ranges[valid]


# ---------------------------------------------------------
# 메인 PF Localizer 클래스
# ---------------------------------------------------------
class PFGlobalLocalizer(Node):

    def __init__(self):
        super().__init__('pf_global_localizer')

        # -------------------------------
        # Parameters
        # -------------------------------
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('map_frame', 'map')

        self.declare_parameter('num_particles', 200)

        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)

        self.declare_parameter('sigma_x', 0.5)
        self.declare_parameter('sigma_y', 0.5)
        self.declare_parameter('sigma_yaw', 0.5)

        # likelihood field params
        self.declare_parameter('sigma_hit', 0.2)
        self.declare_parameter('z_hit', 0.9)
        self.declare_parameter('z_rand', 0.1)

        # motion noise
        self.declare_parameter('motion_noise_lin', 0.01)
        self.declare_parameter('motion_noise_ang', 0.01)

        # -------------------------------
        # Load parameters
        # -------------------------------
        self.map_topic = self.get_parameter('map_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.N = self.get_parameter('num_particles').value

        self.init_x = self.get_parameter('init_x').value
        self.init_y = self.get_parameter('init_y').value
        self.init_yaw = self.get_parameter('init_yaw').value

        self.sigma_x = self.get_parameter('sigma_x').value
        self.sigma_y = self.get_parameter('sigma_y').value
        self.sigma_yaw = self.get_parameter('sigma_yaw').value

        self.sigma_hit = self.get_parameter('sigma_hit').value
        self.sigma_hit2 = self.sigma_hit * self.sigma_hit
        self.z_hit = self.get_parameter('z_hit').value
        self.z_rand = self.get_parameter('z_rand').value

        self.motion_noise_lin = self.get_parameter('motion_noise_lin').value
        self.motion_noise_ang = self.get_parameter('motion_noise_ang').value

        # -------------------------------
        # Internal state
        # -------------------------------
        self.particles = None
        self.weights = None

        self.map_grid: OccupancyGrid | None = None
        self.map_array = None
        self.dist_field = None
        self.map_res = None
        self.map_origin_x = None
        self.map_origin_y = None

        self.last_scan: LaserScan | None = None

        # TF Buffer and Listener for odom->base
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.prev_odom_T = None  # For motion model

        # TF Broadcaster for map->odom
        self.tf_broadcaster = TransformBroadcaster(self)

        # -------------------------------
        # Subscriptions
        # -------------------------------
        self.sub_map = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            1
        )
        self.sub_scan = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        # Timer @ 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            f"PF Global Localizer started\n"
            f"map_topic={self.map_topic}, scan_topic={self.scan_topic}, N={self.N}"
        )

    # ---------------------------------------------------------
    # Map callback (compute distance field)
    # ---------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        self.map_grid = msg
        w = msg.info.width
        h = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))

        # obstacle = 1, free = 0
        occ = np.zeros_like(data, dtype=np.uint8)
        occ[data >= 50] = 1

        # distance transform: free cell → nearest obstacle distance
        dist_cells = distance_transform_edt(1 - occ)
        self.dist_field = dist_cells * msg.info.resolution  # meters

        self.map_array = data
        self.map_res = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        self.get_logger().info(f"Map received: {w}x{h}, distance field built.")

    # ---------------------------------------------------------
    # Scan callback
    # ---------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    # ---------------------------------------------------------
    # Utility: TF lookup odom->base
    # ---------------------------------------------------------
    def lookup_odom_to_base(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        yaw = math.atan2(2 * qw * qz, 1 - 2 * qz * qz)

        T = np.eye(3)
        c, s = math.cos(yaw), math.sin(yaw)
        T[0, 0] = c
        T[0, 1] = -s
        T[1, 0] = s
        T[1, 1] = c
        T[0, 2] = x
        T[1, 2] = y
        return T

    # ---------------------------------------------------------
    # Initialize particles around init pose
    # ---------------------------------------------------------
    def init_particles(self):
        xs = np.random.normal(self.init_x, self.sigma_x, self.N)
        ys = np.random.normal(self.init_y, self.sigma_y, self.N)
        yaws = np.random.normal(self.init_yaw, self.sigma_yaw, self.N)

        self.particles = np.stack((xs, ys, yaws), axis=1)
        self.weights = np.ones(self.N, dtype=np.float64) / self.N

        self.get_logger().info("Particles initialized.")

    # ---------------------------------------------------------
    # Motion update (using odom->base delta)
    # ---------------------------------------------------------
    def motion_update(self, odom_T):
        if self.prev_odom_T is None:
            self.prev_odom_T = odom_T
            return

        # compute odom delta
        T_rel = np.linalg.inv(self.prev_odom_T) @ odom_T
        dx = T_rel[0, 2]
        dy = T_rel[1, 2]
        dyaw = math.atan2(T_rel[1, 0], T_rel[0, 0])

        # noise
        noise_lin = np.random.normal(0.0, self.motion_noise_lin, (self.N, 1))
        noise_ang = np.random.normal(0.0, self.motion_noise_ang, (self.N, 1))

        for i in range(self.N):
            x, y, yaw = self.particles[i]

            c = math.cos(yaw)
            s = math.sin(yaw)

            # body-frame dx,dy → world frame
            wx = c * dx - s * dy
            wy = s * dx + c * dy

            x_new = x + wx + noise_lin[i]
            y_new = y + wy + noise_lin[i]
            yaw_new = yaw + dyaw + noise_ang[i]

            yaw_new = (yaw_new + math.pi) % (2 * math.pi) - math.pi

            self.particles[i] = [x_new, y_new, yaw_new]

        self.prev_odom_T = odom_T

    # ---------------------------------------------------------
    # Measurement update using AMCL-style likelihood field
    # ---------------------------------------------------------
    def measurement_update(self):
        if self.dist_field is None or self.last_scan is None:
            return

        scan = self.last_scan
        angles, ranges = scan_to_angles_ranges(scan, step=10)
        if len(angles) == 0:
            return

        H, W = self.dist_field.shape

        log_w = np.zeros(self.N, dtype=np.float64)

        for i in range(self.N):
            x, y, yaw = self.particles[i]
            c = math.cos(yaw)
            s = math.sin(yaw)

            sum_log = 0.0
            used = 0

            for a, r in zip(angles, ranges):
                theta = yaw + a
                wx = x + r * math.cos(theta)
                wy = y + r * math.sin(theta)

                mx = int((wx - self.map_origin_x) / self.map_res)
                my = int((wy - self.map_origin_y) / self.map_res)

                if mx < 0 or my < 0 or mx >= W or my >= H:
                    continue

                d = self.dist_field[my, mx]

                # likelihood field model
                p_hit = math.exp(-(d * d) / (2 * self.sigma_hit2))
                p = self.z_hit * p_hit + self.z_rand

                sum_log += math.log(p + 1e-12)
                used += 1

            if used == 0:
                log_w[i] = -1e9
            else:
                log_w[i] = sum_log / used

        # normalize (log-sum-exp)
        max_log = np.max(log_w)
        w = np.exp(log_w - max_log)
        w_sum = np.sum(w)
        if w_sum == 0:
            self.weights[:] = 1.0 / self.N
        else:
            self.weights = w / w_sum

        # effective sample size
        neff = 1.0 / np.sum(self.weights ** 2)
        if neff < 0.5 * self.N:
            self.resample_particles()

    # ---------------------------------------------------------
    # Systematic Resampling
    # ---------------------------------------------------------
    def resample_particles(self):
        cdf = np.cumsum(self.weights)
        step = 1.0 / self.N
        r = np.random.uniform(0.0, step)
        idxs = []
        i = 0
        for m in range(self.N):
            u = r + m * step
            while u > cdf[i]:
                i += 1
            idxs.append(i)
        self.particles = self.particles[idxs]
        self.weights = np.ones(self.N, dtype=np.float64) / self.N

    # ---------------------------------------------------------
    # Compute map->odom and publish TF
    # ---------------------------------------------------------
    def publish_map_to_odom(self, stamp, odom_T):
        # particle mean
        xs = self.particles[:, 0]
        ys = self.particles[:, 1]
        yaws = self.particles[:, 2]

        x_mean = np.average(xs, weights=self.weights)
        y_mean = np.average(ys, weights=self.weights)

        sin_y = np.average(np.sin(yaws), weights=self.weights)
        cos_y = np.average(np.cos(yaws), weights=self.weights)
        yaw_mean = math.atan2(sin_y, cos_y)

        # T_map_base
        c = math.cos(yaw_mean)
        s = math.sin(yaw_mean)
        T_map_base = np.eye(3)
        T_map_base[0, 0] = c
        T_map_base[0, 1] = -s
        T_map_base[1, 0] = s
        T_map_base[1, 1] = c
        T_map_base[0, 2] = x_mean
        T_map_base[1, 2] = y_mean

        T_base_odom = np.linalg.inv(odom_T)
        T_map_odom = T_map_base @ T_base_odom

        tx = T_map_odom[0, 2]
        ty = T_map_odom[1, 2]
        yaw_mo = math.atan2(T_map_odom[1, 0], T_map_odom[0, 0])

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(tx)
        t.transform.translation.y = float(ty)
        t.transform.translation.z = 0.0

        qz = math.sin(yaw_mo / 2.0)
        qw = math.cos(yaw_mo / 2.0)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    # ---------------------------------------------------------
    # Timer callback: MCL pipeline
    # ---------------------------------------------------------
    def timer_callback(self):
        if self.map_array is None:
            return
        if self.last_scan is None:
            return

        odom_T = self.lookup_odom_to_base()
        if odom_T is None:
            return

        # initialize
        if self.particles is None:
            self.init_particles()
            self.prev_odom_T = odom_T
            return

        # motion update
        self.motion_update(odom_T)

        # measurement update
        self.measurement_update()

        # publish map->odom
        stamp = self.last_scan.header.stamp
        self.publish_map_to_odom(stamp, odom_T)


# ---------------------------------------------------------
# Main
# ---------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PFGlobalLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

