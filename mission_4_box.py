#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import math

# ================= CONFIG =================

WAYPOINTS = {
    'LEFT':   {'x': -3.0, 'y': 10.0, 'yaw': 1.57},
    'CENTER': {'x': 1.0,  'y': 9.0,  'yaw': 1.57},
    'RIGHT':  {'x': 3.0,  'y': 10.0, 'yaw': 1.57}
}

PUSH_READY_POSES = {
    'LEFT':   {'x': -3.0, 'y': 12.0, 'yaw': 0.0},
    'CENTER': {'x': 0.2,  'y': 15.0, 'yaw': -1.57},
    'RIGHT':  {'x': 3.0,  'y': 12.0, 'yaw': 3.141592}
}

GOAL_ZONE_POSES = {
    'LEFT':   {'x': -0.6, 'y': 12.0, 'yaw': 0.0},
    'CENTER': {'x': -0.2, 'y': 12.5, 'yaw': -1.57},
    'RIGHT':  {'x': 0.6,  'y': 12.0, 'yaw': 3.141592}
}

SEARCH_ORDER = ['LEFT', 'CENTER', 'RIGHT']

POS_THRESH = 0.4
YAW_THRESH = 0.25

V_FWD  = 0.3
V_PUSH = 0.25
K_YAW  = 1.6

TOPIC_POSE   = "/go1_pose"
TOPIC_CMD    = "/cmd_vel"
TOPIC_SPEECH = "/robot_dog/speech"

# ==========================================


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Mission4PushBox(Node):
    def __init__(self):
        super().__init__("mission_4_box")

        self.create_subscription(PoseStamped, TOPIC_POSE, self.pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, TOPIC_CMD, 10)
        self.speech_pub = self.create_publisher(String, TOPIC_SPEECH, 10)

        self.x = None
        self.y = None
        self.yaw = None

        self.search_idx = 0
        self.found_side = None
        self.state = "SEARCH"

        self.get_logger().info("Mission 4 started (minimal search policy)")

        self.create_timer(0.05, self.control_loop)

    def pose_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

        q = msg.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def move_to(self, tx, ty, tyaw, v=V_FWD):
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        yaw_err = normalize_angle(tyaw - self.yaw)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = K_YAW * yaw_err
        self.cmd_pub.publish(cmd)

        return dist, abs(yaw_err)

    def reached(self, tx, ty, tyaw):
        d = math.hypot(tx - self.x, ty - self.y)
        y = abs(normalize_angle(tyaw - self.yaw))
        return d < POS_THRESH and y < YAW_THRESH

    # ---------------- FSM ----------------
    def control_loop(self):
        if self.x is None:
            return

        # ---------- SEARCH ----------
        if self.state == "SEARCH":
            side = SEARCH_ORDER[self.search_idx]
            wp = WAYPOINTS[side]

            self.move_to(wp['x'], wp['y'], wp['yaw'])

            if self.reached(wp['x'], wp['y'], wp['yaw']):
                self.found_side = side
                self.state = "GO_PUSH_READY"
                self.get_logger().info(f"Box found at {side}")
            return

        # ---------- GO PUSH READY ----------
        if self.state == "GO_PUSH_READY":
            pose = PUSH_READY_POSES[self.found_side]
            self.move_to(pose['x'], pose['y'], pose['yaw'])

            if self.reached(pose['x'], pose['y'], pose['yaw']):
                self.state = "PUSH"
                self.get_logger().info("Aligned → pushing")
            return

        # ---------- PUSH ----------
        if self.state == "PUSH":
            goal = GOAL_ZONE_POSES[self.found_side]
            cmd = Twist()
            cmd.linear.x = V_PUSH
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

            if self.reached(goal['x'], goal['y'], goal['yaw']):
                self.stop()
                self.speech_pub.publish(String(data="bark"))
                self.state = "SUCCESS"
                self.get_logger().info("Mission 4 SUCCESS")
            return

        # ---------- SUCCESS ----------
        if self.state == "SUCCESS":
            self.stop()
            return


def main():
    rclpy.init()
    node = Mission4PushBox()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

