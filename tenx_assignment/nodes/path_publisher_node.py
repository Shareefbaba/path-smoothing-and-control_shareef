#!/usr/bin/env python3
"""
tracking_controller_node.py

- Subscribes to:
    - 'time_trajectory' (Float32MultiArray) where data is [x,y,t, x,y,t, ...]
    - 'odom' (nav_msgs/Odometry) for current pose
- Publishes:
    - 'cmd_vel' (geometry_msgs/Twist)

Controller strategy:
- Maintain an index into the trajectory (advance when robot is close)
- Choose a target point ahead (based on lookahead distance)
- Compute heading error and use simple P control on angular velocity:
    omega = k_ang * heading_error
- Compute linear speed:
    - Use feedforward speed from trajectory (if available) or compute desired v from next two samples
    - Scale by cos(heading_error) to slow while turning
    - Clip to [v_min, v_max]
This simple controller is easy to explain and tune.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


def quaternion_to_yaw(q):
    # q: geometry_msgs/Quaternion-like (x,y,z,w)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class TrackingControllerNode(Node):
    def __init__(self):
        super().__init__('tracking_controller_node')

        # Parameters
        self.declare_parameter('lookahead', 0.3)
        self.declare_parameter('k_ang', 2.0)
        self.declare_parameter('v_min', 0.02)
        self.declare_parameter('v_max', 0.4)
        self.declare_parameter('index_advance_dist', 0.15)
        self.declare_parameter('control_rate', 20.0)  # Hz

        self.lookahead = self.get_parameter('lookahead').value
        self.k_ang = self.get_parameter('k_ang').value
        self.v_min = self.get_parameter('v_min').value
        self.v_max = self.get_parameter('v_max').value
        self.index_advance_dist = self.get_parameter('index_advance_dist').value
        self.control_rate = self.get_parameter('control_rate').value

        # State
        self.odom = None  # (x,y,yaw)
        self.trajectory = []  # list of (x,y,t) tuples
        self.traj_idx = 0

        # Subscribers / Publishers
        self.create_subscription(Float32MultiArray, 'time_trajectory', self.traj_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 20)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Control timer
        self.timer = self.create_timer(1.0 / max(1.0, self.control_rate), self.control_loop)

        self.get_logger().info("TrackingControllerNode initialized")

    def traj_cb(self, msg: Float32MultiArray):
        data = np.array(msg.data, dtype=float)
        if data.size == 0:
            self.get_logger().warn("Received empty trajectory")
            return
        if data.size % 3 != 0:
            self.get_logger().warn("time_trajectory data length not multiple of 3")
            return
        pts = data.reshape(-1, 3)
        self.trajectory = [(float(x), float(y), float(t)) for x, y, t in pts]
        self.traj_idx = 0
        self.get_logger().info(f"Received trajectory with {len(self.trajectory)} points")

    def odom_cb(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom = (px, py, yaw)

    @staticmethod
    def angle_diff(a, b):
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    def control_loop(self):
        if self.odom is None or not self.trajectory:
            return

        x, y, yaw = self.odom

        # Advance traj_idx if close to current trajectory point (to avoid getting stuck)
        while self.traj_idx < len(self.trajectory) - 1:
            tx, ty, _ = self.trajectory[self.traj_idx]
            if math.hypot(tx - x, ty - y) < self.index_advance_dist:
                self.traj_idx += 1
            else:
                break

        # Find lookahead target: first point farther than lookahead distance from robot
        look_idx = self.traj_idx
        while look_idx < len(self.trajectory) - 1:
            tx, ty, _ = self.trajectory[look_idx]
            if math.hypot(tx - x, ty - y) >= self.lookahead:
                break
            look_idx += 1

        # clamp
        look_idx = min(look_idx, len(self.trajectory) - 1)
        tx, ty, tt = self.trajectory[look_idx]

        # geometry
        dx = tx - x
        dy = ty - y
        target_yaw = math.atan2(dy, dx)
        err_yaw = self.angle_diff(target_yaw, yaw)

        # feedforward desired speed: approximate using next segment if possible
        desired_v = 0.0
        if look_idx < len(self.trajectory) - 1:
            x0, y0, t0 = self.trajectory[look_idx]
            x1, y1, t1 = self.trajectory[look_idx + 1]
            dt = max(1e-6, t1 - t0)
            ds = math.hypot(x1 - x0, y1 - y0)
            desired_v = ds / dt
        else:
            # last point: slow to zero
            desired_v = self.v_min

        # scale linear velocity by heading error (slow when big heading error)
        v = desired_v * max(0.0, math.cos(err_yaw))
        # ensure clip
        v = max(self.v_min, min(self.v_max, v))

        # angular control (P)
        omega = self.k_ang * err_yaw

        # publish Twist
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    # optional helper: stop robot (can be called on shutdown)
    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()
