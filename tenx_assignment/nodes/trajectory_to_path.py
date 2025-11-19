#!/usr/bin/env python3
"""
trajectory_generator_node.py

- Smooths given 2D waypoints using a B-spline (scipy)
- Samples uniformly by arc-length
- Time-parameterizes path (constant | trapezoidal | curvature_limited)
- Publishes:
    - nav_msgs/Path on 'smoothed_path' (for RViz)
    - std_msgs/Float32MultiArray on 'time_trajectory' as [x,y,t, x,y,t, ...]
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

# requires scipy
try:
    from scipy.interpolate import splprep, splev
except Exception as e:
    raise ImportError("scipy is required: pip install scipy") from e


# ----------------- Helpers (same as earlier) -----------------
def arc_length(pts):
    if len(pts) < 2:
        return np.array([0.0])
    segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(segs)))


def bspline_smooth(waypoints, num_samples=200, smoothing=0.0, degree=3):
    pts = np.asarray(waypoints, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError("waypoints must be shape (N,2)")
    if pts.shape[0] < 2:
        raise ValueError("need at least 2 waypoints")
    k = min(degree, pts.shape[0] - 1)
    tck, _ = splprep([pts[:, 0], pts[:, 1]], s=smoothing, k=k)
    dense = max(1000, num_samples * 6)
    u_dense = np.linspace(0.0, 1.0, dense)
    xy_dense = np.vstack(splev(u_dense, tck, der=0)).T
    s_dense = arc_length(xy_dense)
    total_len = s_dense[-1]
    if total_len <= 0 or not np.isfinite(total_len):
        return np.repeat(xy_dense[:1], num_samples, axis=0)
    s_targets = np.linspace(0.0, total_len, num_samples)
    u_samples = np.interp(s_targets, s_dense, u_dense)
    xs, ys = splev(u_samples, tck, der=0)
    samples = np.vstack((xs, ys)).T
    return samples


def time_parameterize_constant(samples, v=0.2):
    if v <= 0:
        raise ValueError("v must be > 0")
    s = arc_length(samples)
    return s / v


def time_parameterize_trapezoidal(samples, v_max=0.3, a_max=0.5):
    s = arc_length(samples)
    S = s[-1]
    if S <= 0:
        return np.zeros(len(samples), dtype=float)
    t_acc = v_max / a_max
    s_acc = 0.5 * a_max * t_acc ** 2
    if 2 * s_acc >= S:
        t_acc = math.sqrt(S / a_max)
        t_total = 2 * t_acc
        def time_of_s(si):
            if si <= S / 2.0:
                return math.sqrt(2.0 * si / a_max)
            else:
                return t_total - math.sqrt(2.0 * (S - si) / a_max)
    else:
        s_cruise = S - 2.0 * s_acc
        t_cruise = s_cruise / v_max
        def time_of_s(si):
            if si < s_acc:
                return math.sqrt(2.0 * si / a_max)
            elif si <= s_acc + s_cruise:
                return t_acc + (si - s_acc) / v_max
            else:
                return t_acc + t_cruise + (t_acc - math.sqrt(2.0 * (S - si) / a_max))
    return np.array([time_of_s(si) for si in s])


def compute_discrete_curvature(samples):
    pts = np.asarray(samples, dtype=float)
    N = len(pts)
    if N < 3:
        return np.zeros(N)
    kappa = np.zeros(N)
    for i in range(1, N - 1):
        p0, p1, p2 = pts[i - 1], pts[i], pts[i + 1]
        a = np.linalg.norm(p1 - p0)
        b = np.linalg.norm(p2 - p1)
        c = np.linalg.norm(p2 - p0)
        area = abs(np.cross(p1 - p0, p2 - p0)) / 2.0
        denom = a * b * c
        kappa[i] = 4.0 * area / denom if denom > 1e-12 else 0.0
    kappa[0] = kappa[1]
    kappa[-1] = kappa[-2]
    return kappa


def compute_curvature_limited_speed(samples, v_max=0.4, a_lat_max=0.8, v_min=0.05):
    kappa = compute_discrete_curvature(samples)
    v_profile = np.full_like(kappa, v_max, dtype=float)
    eps = 1e-8
    for i, k in enumerate(kappa):
        if abs(k) > eps:
            v_lim = math.sqrt(max(0.0, a_lat_max / (abs(k) + 1e-12)))
            v_profile[i] = min(v_profile[i], v_lim)
    return np.clip(v_profile, v_min, v_max)


def integrate_variable_speed_to_times(samples, v_profile):
    pts = np.asarray(samples, dtype=float)
    N = len(pts)
    if N == 0:
        return np.array([])
    if N == 1:
        return np.array([0.0])
    seg_ds = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    times = np.zeros(N, dtype=float)
    for i in range(0, N - 1):
        vi = max(1e-6, v_profile[i])
        vj = max(1e-6, v_profile[i + 1])
        ds = seg_ds[i]
        dt = 2.0 * ds / (vi + vj)
        times[i + 1] = times[i] + dt
    return times


# --------------------- Node ---------------------
class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')

        # ROS parameters (tune as needed)
        self.declare_parameter('num_samples', 300)
        self.declare_parameter('smoothing', 0.001)
        self.declare_parameter('degree', 3)
        self.declare_parameter('time_mode', 'trapezoidal')  # 'constant'|'trapezoidal'|'curvature_limited'
        self.declare_parameter('v_des', 0.2)
        self.declare_parameter('v_max', 0.4)
        self.declare_parameter('a_max', 0.6)
        self.declare_parameter('a_lat_max', 0.8)
        self.declare_parameter('v_min', 0.05)
        self.declare_parameter('publish_rate', 1.0)

        self.num_samples = self.get_parameter('num_samples').value
        self.smoothing = self.get_parameter('smoothing').value
        self.degree = self.get_parameter('degree').value
        self.time_mode = self.get_parameter('time_mode').value
        self.v_des = self.get_parameter('v_des').value
        self.v_max = self.get_parameter('v_max').value
        self.a_max = self.get_parameter('a_max').value
        self.a_lat_max = self.get_parameter('a_lat_max').value
        self.v_min = self.get_parameter('v_min').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Example waypoints (replace or make dynamic later)
        self.waypoints = [
            (0.0, 0.0),
            (0.8, 0.2),
            (1.6, -0.1),
            (2.5, 0.6),
            (3.0, 1.2),
            (3.8, 1.1),
            (4.5, 0.4),
        ]

        # Publishers
        self.path_pub = self.create_publisher(Path, 'smoothed_path', 10)
        self.traj_pub = self.create_publisher(Float32MultiArray, 'time_trajectory', 10)

        # Build samples
        self.samples = bspline_smooth(self.waypoints, num_samples=self.num_samples,
                                      smoothing=self.smoothing, degree=self.degree)

        # Time-parameterize
        if self.time_mode == 'constant':
            self.times = time_parameterize_constant(self.samples, v=self.v_des)
        elif self.time_mode == 'trapezoidal':
            self.times = time_parameterize_trapezoidal(self.samples, v_max=self.v_max, a_max=self.a_max)
        elif self.time_mode == 'curvature_limited':
            v_profile = compute_curvature_limited_speed(self.samples, v_max=self.v_max,
                                                        a_lat_max=self.a_lat_max, v_min=self.v_min)
            self.times = integrate_variable_speed_to_times(self.samples, v_profile)
        else:
            self.get_logger().warn("Unknown time_mode -> using constant")
            self.times = time_parameterize_constant(self.samples, v=self.v_des)

        # Build trajectory list of (x,y,t)
        self.trajectory = [(float(x), float(y), float(t)) for (x, y), t in zip(self.samples, self.times)]

        # Publish periodically (RViz friendly)
        self.timer = self.create_timer(1.0 / max(0.1, self.publish_rate), self.publish)

        self.get_logger().info(f"Generated trajectory: {len(self.trajectory)} pts, total_time={self.times[-1]:.3f}s")

    def publish(self):
        # Publish smoothed_path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for x, y, _ in self.trajectory:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

        # Publish flat time_trajectory [x,y,t, x,y,t, ...]
        arr = Float32MultiArray()
        flat = []
        for x, y, t in self.trajectory:
            flat.extend([x, y, t])
        arr.data = flat
        self.traj_pub.publish(arr)
        self.get_logger().info(f"Published trajectory ({len(self.trajectory)} pts)")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
