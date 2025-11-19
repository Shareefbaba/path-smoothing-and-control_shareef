#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

import numpy as np
import math
import time

from tenx_assignment.utils.centripetal_spline import centripetal_catmull_rom
from tenx_assignment.utils.time_parameterization_centripetal import generate_time_parameterized_trajectory

class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')

        # parameters
        self.declare_parameter('num_samples', 300)
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('approach_samples', 40)
        self.declare_parameter('approach_threshold', 0.5)
        self.declare_parameter('flip_y', False)
        self.declare_parameter('samples_per_segment', 50)
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('latched_qos', True)
        self.declare_parameter('wait_for_odom', True)     # NEW: wait for odom before publishing
        self.declare_parameter('use_smoothed_path', True) # NEW: prefer /smoothed_path if published

        p = self.get_parameter
        self.num_samples = int(p('num_samples').value)
        self.speed = float(p('speed').value)
        self.approach_samples = int(p('approach_samples').value)
        self.approach_threshold = float(p('approach_threshold').value)
        self.flip_y = bool(p('flip_y').value)
        self.samples_per_segment = int(p('samples_per_segment').value)
        self.waypoints_file = p('waypoints_file').value
        self.latched_qos = bool(p('latched_qos').value)
        self.wait_for_odom = bool(p('wait_for_odom').value)
        self.use_smoothed_path = bool(p('use_smoothed_path').value)

        # QoS for latched trajectory (so controller can start late)
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if self.latched_qos else DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.traj_pub = self.create_publisher(Float32MultiArray, 'trajectory', qos)

        # subscribe odom and optionally smoothed_path
        self.latest_odom = None
        self.odom_sub = self.create_subscription(Odometry, 'odom', self._odom_cb, 10)

        self.smoothed_path = None
        if self.use_smoothed_path:
            # subscribe to /smoothed_path, but not required — we'll fallback to default waypoints
            self.create_subscription(Path, 'smoothed_path', self._smoothed_cb, 10)

        # default waypoints
        self.default_waypoints = [
            (0.0, 0.0),
            (1.0, 2.0),
            (3.0, 3.0),
            (4.0, 0.5),
            (6.0, 2.0),
            (7.0, 0.0)
        ]

        self._published = False
        # keep a timer to attempt publish (safe to call repeatedly)
        self._timer = self.create_timer(0.5, self._attempt_publish)

        self.get_logger().info("Trajectory Generator Node started (wait_for_odom=%s, use_smoothed_path=%s)." %
                               (self.wait_for_odom, self.use_smoothed_path))

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.latest_odom = (float(x), float(y), float(yaw))

        # if we haven't published yet and we're waiting for odom, try now
        if not self._published and self.wait_for_odom:
            self._attempt_publish()

    def _smoothed_cb(self, msg: Path):
        # convert Path to Nx2 numpy array and cache
        pts = []
        for p in msg.poses:
            pts.append((float(p.pose.position.x), float(p.pose.position.y)))
        if len(pts) >= 2:
            self.smoothed_path = np.array(pts, dtype=float)
            self.get_logger().info(f"Received /smoothed_path with {len(pts)} poses.")

            # if we haven't published yet, attempt publishing now that smoothed path exists
            if not self._published:
                self._attempt_publish()

    def _load_waypoints_from_csv(self, path):
        try:
            data = np.loadtxt(path, delimiter=',')
            if data.ndim == 1 and data.size == 2:
                data = data.reshape((1,2))
            waypoints = [(float(row[0]), float(row[1])) for row in data]
            return waypoints
        except Exception as e:
            self.get_logger().warn(f"Failed to load waypoints from CSV '{path}': {e}")
            return None

    def _get_waypoints(self):
        if self.waypoints_file:
            w = self._load_waypoints_from_csv(self.waypoints_file)
            if w and len(w) >= 2:
                return np.array(w, dtype=float)
            else:
                self.get_logger().warn("Waypoints file invalid; falling back to defaults.")
        # if smoothed path available and user asked to prefer it, use it
        if self.use_smoothed_path and self.smoothed_path is not None:
            return self.smoothed_path
        return np.array(self.default_waypoints, dtype=float)

    def _create_approach_segment(self, robot_xy, path_start_xy):
        rx, ry = float(robot_xy[0]), float(robot_xy[1])
        px, py = float(path_start_xy[0]), float(path_start_xy[1])
        t = np.linspace(0.0, 1.0, self.approach_samples, endpoint=False)
        xs = rx + (px - rx) * t
        ys = ry + (py - ry) * t
        return np.column_stack((xs, ys))

    def _attempt_publish(self):
        if self._published:
            return

        # if configured to wait for odom, and odom not yet present, don't publish
        if self.wait_for_odom and (self.latest_odom is None):
            self.get_logger().info_once("Waiting for /odom before publishing trajectory...")
            return

        waypoints = self._get_waypoints()
        if waypoints is None or waypoints.shape[0] < 2:
            self.get_logger().warn("No valid waypoints available to generate trajectory.")
            return

        # generate centripetal Catmull-Rom smooth path
        smooth = centripetal_catmull_rom(waypoints, num_samples=self.num_samples, alpha=0.5,
                                         samples_per_segment=self.samples_per_segment)

        if self.flip_y:
            smooth[:,1] = -smooth[:,1]

        approach_added = False
        full_path = smooth.copy()
        if self.latest_odom is not None:
            rx, ry, ryaw = self.latest_odom
            path_start = smooth[0].copy()
            dist_to_start = math.hypot(path_start[0] - rx, path_start[1] - ry)
            if dist_to_start > self.approach_threshold:
                approach_seg = self._create_approach_segment((rx, ry), path_start)
                full_path = np.vstack((approach_seg, smooth))
                approach_added = True
                self.get_logger().info(f"Approach segment added: robot at ({rx:.3f},{ry:.3f}) -> path start ({path_start[0]:.3f},{path_start[1]:.3f}), dist={dist_to_start:.3f} m")
            else:
                self.get_logger().info("Robot near path start; no approach segment added.")

        # time-parameterize
        traj_list, traj_array = generate_time_parameterized_trajectory(full_path, speed=self.speed)

        flat = []
        for (x,y,t) in traj_list:
            flat += [float(x), float(y), float(t)]
        msg = Float32MultiArray()
        msg.data = flat
        self.traj_pub.publish(msg)
        self._published = True

        self.get_logger().info(f"Published trajectory: {len(traj_list)} points. Approach_added={approach_added}")
        s = traj_list[0]; e = traj_list[-1]
        self.get_logger().info(f"Path start: ({s[0]:.3f},{s[1]:.3f}), end: ({e[0]:.3f},{e[1]:.3f}), total_time={e[2]:.3f}s")

        # stop timer to avoid republishing
        try:
            if self._timer:
                self._timer.cancel()
        except Exception:
            pass

    def destroy_node(self):
        try:
            if self._timer is not None:
                self._timer.cancel()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
