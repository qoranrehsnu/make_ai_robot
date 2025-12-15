#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class Go1PosePublisher(Node):
    """
    Publish estimated pose as /go1_pose

    Topic: /go1_pose
    Type : geometry_msgs/PoseStamped
    Frame: map
    Source TF: map -> base
    """

    def __init__(self):
        super().__init__("go1_pose_publisher")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("publish_rate_hz", 20.0)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        rate = float(self.get_parameter("publish_rate_hz").value)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher
        self.pub = self.create_publisher(PoseStamped, "/go1_pose", 10)

        # Timer
        self.timer = self.create_timer(1.0 / rate, self.timer_cb)

        self.get_logger().info(
            f"[POSE_PUB] Publishing /go1_pose from TF ({self.map_frame} -> {self.base_frame})"
        )

    def timer_cb(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.position.z = tf.transform.translation.z

        msg.pose.orientation = tf.transform.rotation

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Go1PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

