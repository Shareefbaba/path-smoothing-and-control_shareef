#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

import numpy as np
import math
import time

# Import your existing spline + time parameterization utilities
# Ensure these modules are on PYTHONPATH / in same package
from tenx_assignment.utils.centripetal_spline import centripetal_catmull_rom
from tenx_assignment.utils.time_parameterization_centripetal import generate_time_parameterized_trajectory

def rotation_matrix(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s],
                     [s,  c]])

def vector_angle(dx, dy):
    return math.atan2(dy, dx)


class TrajectoryGeneratorNode(Node):
    """
    Trajectory generator that can prepend an 'approach' segment from the current /odom pose
    to the first waypoint, then append the smooth trajectory computed from given waypoints.

    Behavior (Option B):
      - If an /odom message is available and the robot is further than approach_threshold
        from the first waypoint, create a straight approach segment (line) from robot_pos -> path_start.
      - Concatenate approach segment + smooth path, time-parameterize, publish as Float32MultiArray.
    """
    def __init__(self):
        super().__init__('trajectory_generator_node')

        # params
        self.declare_parameter('num_samples', 300)            # samples for smooth path
        self.declare_parameter('speed', 0.2)                 # cruising speed (m/s)
        self.declare_parameter('approach_samples', 40)       # samples for approach segment
        self.declare_parameter('approach_threshold', 0.5)    # meters; if robot farther than this, add approach
        self.declare_parameter('flip_y', False)              # if true, flip y-axis of generated path
        self.declare_parameter('samples_per_segment', 50)    # internal catmull sampling
        self.declare_parameter('waypoints_file', '')         # optional CSV of waypoints x,y
        self.declare_parameter('align_to_odom', False)       # not used here, left for compatibility
        self.declare_parameter('latched_qos', True)          # publish transient_local

        p = self.get_parameter
        self.num_samples = int(p('num_samples').value)
        self.speed = float(p('speed').value)
        self.approach_samples = int(p('approach_samples').value)
        self.approach_threshold = float(p('approach_threshold').value)
        self.flip_y = bool(p('flip_y').value)
        self.samples_per_segment = int(p('samples_per_segment').value)
        self.waypoints_file = p('waypoints_file').value
        self.latched_qos = bool(p('latched_qos').value)

        # QoS for latched trajectory
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if self.latched_qos else DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.traj_pub = self.create_publisher(Float32MultiArray, 'trajectory', qos)

        # odom sub to get robot pose for approach
        self.latest_odom = None
        self.odom_sub = self.create_subscription(Odometry, 'odom', self._odom_cb, 10)

        # default waypoints (if none supplied)
        self.default_waypoints = [
            (0.0, 0.0),
            (-2.45, 1.54),
            (-3.7, 4.02),
            (-7.0, 0.0),
            (-7.3, 13.7)
        ]

        self._published = False
        # one-shot timer to publish soon after startup
        self._timer = self.create_timer(0.8, self._publish_once)

        self.get_logger().info("Trajectory Generator Node (Option B: approach-prepend) started.")

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.latest_odom = (float(x), float(y), float(yaw))

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
        # priority: CSV -> default
        if self.waypoints_file:
            w = self._load_waypoints_from_csv(self.waypoints_file)
            if w and len(w) >= 2:
                return w
            else:
                self.get_logger().warn("Waypoints file invalid; falling back to default waypoints.")

        return self.default_waypoints

    def _create_approach_segment(self, robot_xy, path_start_xy):
        """
        Create a linear segment from robot_xy -> path_start_xy with self.approach_samples points.
        Returns Nx2 numpy array.
        """
        rx, ry = float(robot_xy[0]), float(robot_xy[1])
        px, py = float(path_start_xy[0]), float(path_start_xy[1])
        # simple straight-line interpolation including path_start as final point
        t = np.linspace(0.0, 1.0, self.approach_samples, endpoint=False)
        xs = rx + (px - rx) * t
        ys = ry + (py - ry) * t
        seg = np.column_stack((xs, ys))
        return seg

    def _publish_once(self):
        if self._published:
            try:
                if self._timer is not None:
                    self._timer.cancel()
            except Exception:
                pass
            return

        waypoints = self._get_waypoints()
        waypoints = np.array(waypoints, dtype=float)

        # generate smooth path using centripetal Catmull-Rom
        smooth = centripetal_catmull_rom(waypoints, num_samples=self.num_samples, alpha=0.5, samples_per_segment=self.samples_per_segment)

        # optional flip_y (if your sim uses different handedness)
        if self.flip_y:
            smooth[:,1] = -smooth[:,1]

        # Decide whether to prepend approach segment
        full_path = smooth.copy()
        approach_added = False
        if self.latest_odom is not None:
            rx, ry, ryaw = self.latest_odom
            path_start = smooth[0].copy()
            dist_to_start = math.hypot(path_start[0] - rx, path_start[1] - ry)
            if dist_to_start > self.approach_threshold:
                # create approach segment and prepend
                approach_seg = self._create_approach_segment((rx, ry), path_start)
                # Concatenate approach segment + smooth path (ensure no duplicate point at seam)
                full_path = np.vstack((approach_seg, smooth))
                approach_added = True
                self.get_logger().info(f"Approach segment added: robot at ({rx:.3f},{ry:.3f}) -> path start ({path_start[0]:.3f},{path_start[1]:.3f}), dist={dist_to_start:.3f} m")
            else:
                self.get_logger().info("Robot already near path start; no approach segment needed.")
        else:
            self.get_logger().warn("No /odom received yet; publishing un-prepended path (robot must drive to path start manually).")

        # time-parameterize full_path
        traj_list, traj_array = generate_time_parameterized_trajectory(full_path, speed=self.speed)

        # flatten into Float32MultiArray data (x,y,t) triples
        flat = []
        for (x,y,t) in traj_list:
            flat += [float(x), float(y), float(t)]
        msg = Float32MultiArray()
        msg.data = flat
        self.traj_pub.publish(msg)
        self._published = True

        self.get_logger().info(f"Published trajectory: {len(traj_list)} points (flat len={len(flat)}). Approach_added={approach_added}")
        start = traj_list[0]
        end = traj_list[-1]
        self.get_logger().info(f"Path start: ({start[0]:.3f}, {start[1]:.3f}), end: ({end[0]:.3f}, {end[1]:.3f}), total_time={end[2]:.3f}s")

        # cancel timer
        try:
            if self._timer is not None:
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
