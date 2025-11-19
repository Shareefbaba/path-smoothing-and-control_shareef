#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

import numpy as np
import math
import time

try:
    from scipy.spatial import cKDTree as KDTree
except Exception:
    KDTree = None


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0*(qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0*(qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)


class TrajectoryControllerNode(Node):
    def __init__(self):
        super().__init__('trajectory_controller_node')

        # Parameters (tunable)
        self.declare_parameter('v_cruise', 0.50)             # base cruising speed (m/s)
        self.declare_parameter('v_multiplier', 1.0)         # multiplier applied to v_cruise (for "fast" runs)
        self.declare_parameter('fast_mode', False)          # if true, multiply speed by v_multiplier
        self.declare_parameter('max_v', 1.2)                # safety cap on linear speed (m/s)
        self.declare_parameter('lookahead', 0.6)
        self.declare_parameter('lookahead_gain', 1.0)
        self.declare_parameter('lp_omega_alpha', 0.85)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('max_omega', 3.0)
        self.declare_parameter('min_v_when_turn', 0.10)
        self.declare_parameter('adaptive_heading_k', 1.5)
        self.declare_parameter('cmd_rate', 20.0)
        self.declare_parameter('require_recent_odom', True)  # if True require odom timestamp check
        self.declare_parameter('odom_fresh_secs', 1.0)      # how recent odom must be (seconds)

        # read params
        self.v_cruise = float(self.get_parameter('v_cruise').value)
        self.v_multiplier = float(self.get_parameter('v_multiplier').value)
        self.fast_mode = bool(self.get_parameter('fast_mode').value)
        self.max_v = float(self.get_parameter('max_v').value)
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.lookahead_gain = float(self.get_parameter('lookahead_gain').value)
        self.lp_omega_alpha = float(self.get_parameter('lp_omega_alpha').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.max_omega = float(self.get_parameter('max_omega').value)
        self.min_v_when_turn = float(self.get_parameter('min_v_when_turn').value)
        self.adaptive_heading_k = float(self.get_parameter('adaptive_heading_k').value)
        self.cmd_rate = float(self.get_parameter('cmd_rate').value)
        self.require_recent_odom = bool(self.get_parameter('require_recent_odom').value)
        self.odom_fresh_secs = float(self.get_parameter('odom_fresh_secs').value)

        # Internal ready flag: will be True only if both trajectory received and odom is present/fresh
        self.ready = False

        # Create subscriber for /trajectory using TRANSIENT_LOCAL QoS so late subscribers receive the last published traj
        traj_qos = QoSProfile(depth=1)
        traj_qos.reliability = ReliabilityPolicy.RELIABLE
        traj_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        traj_qos.history = HistoryPolicy.KEEP_LAST

        self.create_subscription(
            Float32MultiArray,
            '/trajectory',
            self.trajectory_array_cb,
            traj_qos
        )

        # odom subscription
        # Use default/reliable qos for odom
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            20
        )

        # Publisher for cmd_vel - keep RELIABLE QoS so IsaacSim receives it
        cmd_qos = QoSProfile(depth=5)
        cmd_qos.reliability = ReliabilityPolicy.RELIABLE
        cmd_qos.history = HistoryPolicy.KEEP_LAST
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', cmd_qos)

        # Internal states
        self.traj_pts = np.zeros((0, 3))   # Nx3 (x,y,t)
        self.traj_kdtree = None
        self.odom_pose = None              # (x,y,yaw)
        self.odom_stamp = None             # rclpy.time.Time or float seconds for freshness check
        self.goal_reached = False
        self.omega_prev = 0.0

        # Control loop timer
        self.create_timer(
            1.0 / max(0.001, self.cmd_rate),
            self.control_loop
        )

        self.get_logger().info("TrajectoryControllerNode started. Waiting for /trajectory and /odom.")

    # ----------------------------------------------------------
    # Trajectory from Float32MultiArray: [x0,y0,t0, x1,y1,t1, ...]
    # ----------------------------------------------------------
    def trajectory_array_cb(self, msg: Float32MultiArray):
        data = np.asarray(msg.data, dtype=float)
        if data.size == 0:
            self.get_logger().warn("Empty /trajectory received.")
            return

        if data.size % 3 != 0:
            self.get_logger().warn(f"Array length {data.size} not multiple of 3. Truncating.")
        n = data.size // 3
        data = data[:3*n]
        pts = data.reshape((n, 3))

        self.traj_pts = pts
        if self.traj_pts.shape[0] >= 2 and KDTree is not None:
            try:
                self.traj_kdtree = KDTree(self.traj_pts[:, 0:2])
            except Exception as e:
                self.get_logger().warn(f"KDTree build failed: {e}")
                self.traj_kdtree = None
        else:
            self.traj_kdtree = None

        self.goal_reached = False

        # Only set ready if we have recent odom (or if odom already present)
        if self.odom_pose is None:
            self.ready = False
            self.get_logger().info(f"Trajectory loaded ({n} pts) but waiting for /odom to enable motion.")
        else:
            if self.require_recent_odom:
                # check how recent odom is (we store odom_stamp as float seconds)
                try:
                    now = self.get_clock().now().nanoseconds * 1e-9
                except Exception:
                    now = time.time()
                odom_time = self.odom_stamp if self.odom_stamp is not None else 0.0
                age = now - odom_time
                if age > self.odom_fresh_secs:
                    self.ready = False
                    self.get_logger().info(f"Trajectory loaded but odom too old (age={age:.3f}s). Waiting for fresh /odom.")
                    return
            self.ready = True
            self.get_logger().info(f"Loaded trajectory with {n} points. Controller is now READY.")

    # ----------------------------------------------------------
    # Odom callback
    # ----------------------------------------------------------
    def odom_cb(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_pose = (float(px), float(py), float(yaw))

        # store timestamp for freshness checks
        try:
            # convert ROS time to float seconds
            sec = msg.header.stamp.sec
            nsec = msg.header.stamp.nanosec
            self.odom_stamp = float(sec) + float(nsec) * 1e-9
        except Exception:
            self.odom_stamp = time.time()

        # If a trajectory is already loaded but controller wasn't ready because odom missing/old, enable if fresh
        if self.traj_pts.shape[0] > 0 and not self.ready:
            if self.require_recent_odom:
                try:
                    now = self.get_clock().now().nanoseconds * 1e-9
                except Exception:
                    now = time.time()
                age = now - self.odom_stamp
                if age <= self.odom_fresh_secs:
                    self.ready = True
                    self.get_logger().info("Fresh odom received — enabling controller (trajectory already loaded).")
            else:
                self.ready = True
                self.get_logger().info("Odom received — enabling controller (trajectory already loaded).")

    # ----------------------------------------------------------
    # Pure-pursuit control loop
    # ----------------------------------------------------------
    def control_loop(self):
        # Safety: do not move until we have an active trajectory AND odom available
        if not self.ready:
            # publish stop to be safe
            self._publish_stop()
            return

        if self.odom_pose is None:
            # wait for odom
            self._publish_stop()
            return

        if self.traj_pts.shape[0] == 0:
            self._publish_stop()
            return

        if self.goal_reached:
            self._publish_stop()
            return

        x, y, yaw = self.odom_pose

        # nearest trajectory index
        if self.traj_kdtree:
            try:
                _, idx = self.traj_kdtree.query([x, y])
                idx = int(idx)
            except Exception:
                d = np.hypot(self.traj_pts[:,0] - x, self.traj_pts[:,1] - y)
                idx = int(np.argmin(d))
        else:
            d = np.hypot(self.traj_pts[:,0] - x, self.traj_pts[:,1] - y)
            idx = int(np.argmin(d))

        # check final goal stop
        gx, gy = self.traj_pts[-1, 0], self.traj_pts[-1, 1]
        dist_goal = math.hypot(gx - x, gy - y)
        if dist_goal <= self.goal_tolerance:
            self.get_logger().info(f"Reached goal (d={dist_goal:.3f}). Stopping.")
            self.goal_reached = True
            self._publish_stop()
            return

        # compute dynamic lookahead
        Ld = max(self.lookahead, self.v_cruise * self.lookahead_gain)
        acc = 0.0
        i = idx
        while i + 1 < self.traj_pts.shape[0]:
            seg = math.hypot(
                self.traj_pts[i+1,0] - self.traj_pts[i,0],
                self.traj_pts[i+1,1] - self.traj_pts[i,1]
            )
            acc += seg
            i += 1
            if acc >= Ld:
                break

        lx, ly = float(self.traj_pts[i,0]), float(self.traj_pts[i,1])

        # transform to robot frame
        dx = lx - x
        dy = ly - y
        x_r = math.cos(-yaw)*dx - math.sin(-yaw)*dy
        y_r = math.sin(-yaw)*dx + math.cos(-yaw)*dy

        # pure pursuit curvature
        if abs(x_r) < 1e-6:
            kappa = 0.0
        else:
            kappa = 2.0 * y_r / (Ld**2)

        # heading error
        heading_error = math.atan2(y_r, max(1e-6, x_r))
        heading_scale = math.exp(-self.adaptive_heading_k * abs(heading_error))

        # commanded speed: allow fast_mode multiplier and cap by max_v
        v_cmd = self.v_cruise
        if self.fast_mode:
            v_cmd *= self.v_multiplier
        v = v_cmd * heading_scale
        v = max(self.min_v_when_turn, v) if abs(heading_error) > 0.7 else v
        # enforce safety cap
        v = min(v, self.max_v)

        # angular velocity
        omega = v * kappa
        omega = max(-self.max_omega, min(self.max_omega, omega))

        # low-pass filter for omega
        omega_f = self.lp_omega_alpha * self.omega_prev + (1.0 - self.lp_omega_alpha) * omega
        self.omega_prev = omega_f

        # publish cmd_vel
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(omega_f)
        self.cmd_pub.publish(twist)

    # ----------------------------------------------------------
    def _publish_stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
