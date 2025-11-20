#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

import numpy as np
from builtin_interfaces.msg import Time

# replace with your actual spline import path
from tenx_assignment.utils.centripetal_spline import centripetal_catmull_rom

class PathSmootherNode(Node):
    def __init__(self):
        super().__init__('path_smoother_node')

        # params
        self.declare_parameter('num_samples', 300)
        self.declare_parameter('alpha', 0.5)
        self.declare_parameter('frame_id', 'odom')         # keep it consistent with odom
        self.declare_parameter('latched_qos', True)       # publish /trajectory latched
        self.declare_parameter('reverse_path', False)     # flip path order if needed
        self.declare_parameter('publish_trajectory_topic', True)  # publish Float32MultiArray for controller

        self.num_samples = int(self.get_parameter('num_samples').value)
        self.alpha = float(self.get_parameter('alpha').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.latched_qos = bool(self.get_parameter('latched_qos').value)
        self.reverse_path = bool(self.get_parameter('reverse_path').value)
        self.publish_trajectory_topic = bool(self.get_parameter('publish_trajectory_topic').value)

        # publishers
        # Path for RViz (normal QoS)
        self.path_pub = self.create_publisher(Path, '/smoothed_path', 10)

        # /trajectory (Float32MultiArray) latched if requested
        traj_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if self.latched_qos else DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE
        )
        if self.publish_trajectory_topic:
            self.traj_pub = self.create_publisher(Float32MultiArray, '/trajectory', traj_qos)
        else:
            self.traj_pub = None

        # single-shot timer to run once after startup (allow a little time for TF/odom)
        self.has_run = False
        self.create_timer(0.6, self._run_once)

        # default waypoint list (edit as needed)
        self.waypoints = np.array([
            (0.0, 0.0),
            (-2.45, 1.54),
            (-3.7, 4.02),
            (-7.0, 0.0),
            (-7.3, 13.7)
        ], dtype=float)

        self.get_logger().info("PathSmootherNode ready.")

    def _run_once(self):
        if self.has_run:
            return
        self.has_run = True

        self.get_logger().info(f"Generating spline for {len(self.waypoints)} waypoints...")
        smooth_xy = centripetal_catmull_rom(
            pts=self.waypoints,
            num_samples=self.num_samples,
            alpha=self.alpha
        )

        # final sample exact to last waypoint
        if smooth_xy.shape[0] > 0:
            smooth_xy[-1, 0] = float(self.waypoints[-1,0])
            smooth_xy[-1, 1] = float(self.waypoints[-1,1])

        # optionally reverse if your result is reversed
        if self.reverse_path:
            smooth_xy = smooth_xy[::-1]

        # publish nav_msgs/Path for RViz
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in smooth_xy:
            p = PoseStamped()
            p.header.frame_id = self.frame_id
            p.header.stamp = self.get_clock().now().to_msg()   # per-point stamp not used by your controller but set to now
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.position.z = 0.0
            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = 0.0
            p.pose.orientation.w = 1.0
            path_msg.poses.append(p)

        self.path_pub.publish(path_msg)
        self.get_logger().info("Published /smoothed_path (nav_msgs/Path)")

        # publish Float32MultiArray /trajectory (x,y,t) so controller will accept it
        if self.traj_pub is not None:
            # simple constant speed time parameterization: compute cumulative distance, t = dist / speed
            speed = 0.25  # default speed used for time stamps; controller uses its own v_cruise too
            dists = np.sqrt(np.sum(np.diff(smooth_xy, axis=0)**2, axis=1))
            cum = np.concatenate(([0.0], np.cumsum(dists)))
            times = cum / max(1e-6, speed)

            flat = []
            for i, (x,y) in enumerate(smooth_xy):
                t = float(times[i]) if i < times.shape[0] else float(times[-1])
                flat += [float(x), float(y), t]

            msg = Float32MultiArray()
            msg.data = flat
            self.traj_pub.publish(msg)
            self.get_logger().info(f"Published /trajectory (Float32MultiArray) with {len(smooth_xy)} points (latched={self.latched_qos})")

def main(args=None):
    rclpy.init(args=args)
    node = PathSmootherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
