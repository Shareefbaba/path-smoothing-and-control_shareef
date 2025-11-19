#!/usr/bin/env python3
"""
tracking_controller_node.py

Robust tracking controller for a 2D unicycle-like robot.

Features:
- Pure-pursuit curvature control (recommended)
- Adaptive slowdown of linear speed when heading error is large
- Low-pass filter on angular velocity to smooth commands
- QoS tuned for Isaac Sim (BEST_EFFORT on cmd_vel)
- Parameters are declared so you can tune at runtime via ros2 param

Default recommended params (good starting point for Isaac Sim):
  lookahead_base = 0.6
  lookahead_gain = 0.6
  speed = 0.15
  min_speed = 0.03
  kp_heading = 0.8
  max_omega = 0.8
  w_filter_tau = 0.08
  use_pure_pursuit = True

Usage:
  ros2 run <your_package> tracking_controller_node
  (or run directly: python3 tracking_controller_node.py)
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import math
import time

# ----------------- Helpers -----------------
def wrap_pi(a: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def compute_arc_lengths(path: np.ndarray) -> np.ndarray:
    """Return cumulative arc lengths for Nx2 path."""
    if path is None or path.shape[0] == 0:
        return np.array([0.0], dtype=float)
    d = np.zeros(path.shape[0], dtype=float)
    for i in range(1, path.shape[0]):
        dx = float(path[i, 0] - path[i - 1, 0])
        dy = float(path[i, 1] - path[i - 1, 1])
        d[i] = d[i - 1] + math.hypot(dx, dy)
    return d


def find_point_at_arc_distance(arcs: np.ndarray, target_dist: float) -> int:
    """Return index of first arc >= target_dist."""
    if arcs is None or len(arcs) == 0:
        return 0
    idx = int(np.searchsorted(arcs, target_dist))
    return min(idx, len(arcs) - 1)


def nearest_point_index_local(path: np.ndarray, x: float, y: float, last_idx: int = 0, window: int = 60) -> int:
    """
    Find nearest index to (x,y) within a local window around last_idx to avoid global search.
    """
    if path is None or path.shape[0] == 0:
        return 0
    n = path.shape[0]
    start = max(0, last_idx - 10)
    end = min(n, last_idx + window)
    seg = path[start:end]
    if seg.shape[0] == 0:
        return 0
    d = np.hypot(seg[:, 0] - x, seg[:, 1] - y)
    rel = int(np.argmin(d))
    return start + rel


# ----------------- Node -----------------
class TrackingControllerNode(Node):
    def __init__(self):
        super().__init__('tracking_controller_node')

        # ---------- Declare parameters (tunable at runtime) ----------
        self.declare_parameter('lookahead_base', 0.6)
        self.declare_parameter('lookahead_gain', 0.6)
        self.declare_parameter('speed', 0.15)
        self.declare_parameter('min_speed', 0.03)
        self.declare_parameter('kp_heading', 0.8)
        self.declare_parameter('max_omega', 0.8)
        self.declare_parameter('control_dt', 0.05)
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('use_pure_pursuit', True)
        self.declare_parameter('w_filter_tau', 0.08)
        self.declare_parameter('debug_log_every', 10)
        self.declare_parameter('lookahead_window', 60)

        p = self.get_parameter
        self.lookahead_base = float(p('lookahead_base').value)
        self.lookahead_gain = float(p('lookahead_gain').value)
        self.speed = float(p('speed').value)
        self.min_speed = float(p('min_speed').value)
        self.kp_heading = float(p('kp_heading').value)
        self.max_omega = float(p('max_omega').value)
        self.control_dt = float(p('control_dt').value)
        self.goal_tolerance = float(p('goal_tolerance').value)
        self.use_pure_pursuit = bool(p('use_pure_pursuit').value)
        self.w_filter_tau = float(p('w_filter_tau').value)
        self.debug_log_every = int(p('debug_log_every').value)
        self.lookahead_window = int(p('lookahead_window').value)

        # ---------- QoS for cmd_vel (Isaac Sim typically wants BEST_EFFORT) ----------
        qos_cmd = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # ---------- Subscribers & Publishers ----------
        self.traj_sub = self.create_subscription(Float32MultiArray, 'trajectory', self.trajectory_cb, 10)
        # generator often latches as transient_local; subscription default is fine.
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos_cmd)

        # ---------- state ----------
        self.path = np.zeros((0, 2), dtype=float)
        self.arc_lengths = np.array([0.0], dtype=float)
        self.last_near_idx = 0
        self._finished = False

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # angular filter state
        self.w_filter_state = 0.0
        self.last_time = self.get_clock().now()

        # timer
        self.loop_count = 0
        self.timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info("🚀 Tracking Controller Node Started — READY")
        self.get_logger().info(f"Params: lookahead_base={self.lookahead_base}, lookahead_gain={self.lookahead_gain}, speed={self.speed}, min_speed={self.min_speed}, max_omega={self.max_omega}, w_tau={self.w_filter_tau}")

    # ----------------- Callbacks -----------------
    def trajectory_cb(self, msg: Float32MultiArray):
        try:
            data = np.array(msg.data, dtype=float)
            if data.size == 0:
                self.get_logger().warn("❗ Received EMPTY trajectory")
                return
            # reshape to Nx3 or Nx2 (x,y,t) or (x,y)
            if data.size % 3 == 0:
                pts = data.reshape(-1, 3)[:, :2]
            elif data.size % 2 == 0:
                pts = data.reshape(-1, 2)
            else:
                # fallback: try to infer pairs
                pts = data.reshape(-1, 2)
            self.path = np.array(pts, dtype=float)
            self.arc_lengths = compute_arc_lengths(self.path)
            self.last_near_idx = 0
            self._finished = False
            self.get_logger().info(f"📌 Trajectory loaded: {len(self.path)} pts, length={self.arc_lengths[-1]:.3f} m")
        except Exception as e:
            self.get_logger().error(f"Failed to parse trajectory message: {e}")

    def odom_cb(self, msg: Odometry):
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = float(math.atan2(siny_cosp, cosy_cosp))

    # ----------------- Control Loop -----------------
    def control_loop(self):
        # nothing to do until path loaded
        if self.path is None or self.path.shape[0] == 0:
            return
        if self._finished:
            self.publish_stop()
            return

        # compute dt for filter
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = self.control_dt
        self.last_time = now

        # nearest point on path (local window)
        near = nearest_point_index_local(self.path, self.robot_x, self.robot_y, self.last_near_idx, window=self.lookahead_window)
        self.last_near_idx = int(near)

        # dynamic lookahead (depends on speed)
        L = float(self.lookahead_base + self.lookahead_gain * self.speed)
        target_dist = float(self.arc_lengths[near] + L)
        target_idx = find_point_at_arc_distance(self.arc_lengths, target_dist)

        # ensure target_idx moves forward at least one
        if target_idx <= near and near < (len(self.path) - 1):
            target_idx = min(near + 1, len(self.path) - 1)

        gx, gy = float(self.path[target_idx, 0]), float(self.path[target_idx, 1])

        # geometry
        angle_to_goal = math.atan2(gy - self.robot_y, gx - self.robot_x)
        heading_error = wrap_pi(angle_to_goal - self.robot_yaw)

        # distance to final goal
        fx, fy = float(self.path[-1, 0]), float(self.path[-1, 1])
        dist_to_goal = math.hypot(fx - self.robot_x, fy - self.robot_y)

        # nominal linear speed (may be scaled down by heading)
        v_nom = float(self.speed)
        # compute adaptive slowdown factor based on heading to lookahead
        # when heading large (~90deg) -> slow_factor -> 0, so robot turns in place
        slow_factor = max(0.0, math.cos(min(abs(heading_error), math.pi/2)))
        v = max(self.min_speed, v_nom * slow_factor)

        # compute angular command
        if self.use_pure_pursuit:
            alpha = wrap_pi(angle_to_goal - self.robot_yaw)  # same as heading_error
            L_eff = max(L, 1e-3)
            kappa = 2.0 * math.sin(alpha) / L_eff
            w_cmd = v * kappa
        else:
            w_cmd = float(self.kp_heading * heading_error)

        # saturate angular velocity command
        w_cmd = max(min(w_cmd, self.max_omega), -self.max_omega)

        # low-pass filter on w (first-order)
        tau = max(1e-6, float(self.w_filter_tau))
        alpha_f = dt / (tau + dt)
        self.w_filter_state = (1.0 - alpha_f) * self.w_filter_state + alpha_f * w_cmd
        w = float(self.w_filter_state)

        # check goal
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info("🎉 Goal Reached! Stopping.")
            self._finished = True
            self.publish_stop()
            return

        # publish command
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

        # debug logging
        self.loop_count += 1
        if (self.loop_count % max(1, self.debug_log_every)) == 0:
            self.get_logger().info(
                f"CMD → v={v:.2f}, w={w:.2f}, near={near}, tgt={target_idx}, dist_goal={dist_to_goal:.3f}, head_err={heading_error:.3f}, L={L:.2f}"
            )

    def publish_stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
