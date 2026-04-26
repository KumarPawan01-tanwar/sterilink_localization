#!/usr/bin/env python3
"""
sterilink_localization / odom_icp  —  STERILINK ICP Odometry Node
==================================================================
2D POINT-TO-LINE ICP scan-matching odometry for the STERILINK 1:5 model car.

Hardware handled:
    * YDLIDAR G4 mounted 180° backward  → handled by car_description TF
      (angle_offset = 0; the TF 7/base_link→7/laser_frame does the rotation)
    * Robot body visible at 0.13–0.25 m  → range_min_clip = 0.26 m
    * YDLIDAR reports 0.0 for clear sky  → explicit zero filter
    * Featureless corridors cause jumps  → max_jump_dist = 2.0 m

ICP method: Point-to-Line (Pl-ICP)
    Unlike point-to-point ICP, this projects each source point onto the
    local tangent line of the target surface.  This correctly constrains
    rotation even when driving along flat walls.

Outputs (nav_msgs/Odometry):
    * pose.position         → x, y          (odom frame)
    * pose.orientation      → yaw           (quaternion)
    * twist.linear          → vx, vy        (base_link frame, REP-103)
    * twist.angular.z       → yaw rate      (base_link frame)

Subscribes : /scan                       (sensor_msgs/LaserScan)
Publishes  : /modelcar<car_id>/odom_icp  (nav_msgs/Odometry)
TF         : odom → <car_id>/base_link   (gated by publish_tf param)
"""
import os
import math
import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros


DEFAULT_CAR_ID = -1


# ============================================================================
# Helpers
# ============================================================================

def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (rad) to a geometry_msgs/Quaternion (roll=pitch=0)."""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=float(math.sin(yaw / 2.0)),
        w=float(math.cos(yaw / 2.0)),
    )


def normalize_angle(a):
    """Wrap angle to [-π, π]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def make_T(dx, dy, dtheta):
    """Build a 3×3 SE(2) homogeneous transform."""
    c, s = np.cos(dtheta), np.sin(dtheta)
    return np.array([[c, -s, dx],
                     [s,  c, dy],
                     [0,  0,  1]], dtype=np.float64)


def extract_pose(T):
    """Extract (x, y, θ) from a 3×3 SE(2) matrix."""
    return T[0, 2], T[1, 2], np.arctan2(T[1, 0], T[0, 0])


# ============================================================================
# Point-to-Line ICP  (much better yaw estimation than point-to-point)
# ============================================================================

def estimate_normals(points, k=5):
    """
    Estimate 2D surface normals from a point cloud using local PCA.
    For each point, fit a line through its k nearest neighbours;
    the normal is the eigenvector with the smallest eigenvalue.
    """
    tree = cKDTree(points)
    _, idx = tree.query(points, k=k, workers=-1)
    normals = np.zeros_like(points)
    for i in range(len(points)):
        neighbours = points[idx[i]]
        centroid = neighbours.mean(axis=0)
        cov = (neighbours - centroid).T @ (neighbours - centroid)
        eigvals, eigvecs = np.linalg.eigh(cov)
        # smallest eigenvalue → normal direction
        normals[i] = eigvecs[:, 0]
    return normals


def icp_point_to_line(source, target,
                      max_iter=50, conv_thresh=1e-4,
                      max_corr_dist=0.2, min_points=50,
                      normal_k=5):
    """
    Point-to-Line 2D ICP.

    Instead of minimising point-to-point distance, this minimises the
    distance from each source point to the tangent line at its closest
    target point.  This correctly constrains both translation AND
    rotation, even along flat walls.

    Returns (T_accum, converged).
    T_accum aligns source → target (= robot displacement when
    source = current scan, target = previous scan).
    """
    src = source.copy()
    T_accum = np.eye(3, dtype=np.float64)

    # precompute target tree and normals (target doesn't change)
    tree = cKDTree(target)
    normals = estimate_normals(target, k=normal_k)

    for _ in range(max_iter):
        dists, idx = tree.query(src, k=1, workers=-1)
        valid = dists < max_corr_dist
        if valid.sum() < min_points:
            return T_accum, False

        src_v = src[valid]
        tgt_v = target[idx[valid]]
        nrm_v = normals[idx[valid]]

        # ---- solve point-to-line minimisation (closed form) ----
        # Minimise  sum_i  [ n_i · (R·s_i + t - t_i) ]²
        # Linearise R ≈ I + [[0,-dθ],[dθ,0]]  for small dθ
        # This gives a 3×3 linear system  A·x = b  where x = [dx, dy, dθ]

        n = len(src_v)
        A = np.zeros((3, 3), dtype=np.float64)
        b = np.zeros(3, dtype=np.float64)

        for i in range(n):
            sx, sy = src_v[i]
            nx, ny = nrm_v[i]
            # derivative of R·s w.r.t. dθ  (at dθ=0):  [-sy, sx]
            # projection onto normal: a = nx*1 (for dx), ny*1 (for dy),
            #   nx*(-sy) + ny*(sx) (for dθ)
            a = np.array([nx, ny, nx * (-sy) + ny * sx])
            # residual projected onto normal
            ex, ey = src_v[i] - tgt_v[i]
            ei = nx * ex + ny * ey

            A += np.outer(a, a)
            b -= a * ei

        # solve
        try:
            x = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            return T_accum, False

        dx, dy, dtheta = x[0], x[1], x[2]

        # apply incremental transform
        T_inc = make_T(dx, dy, dtheta)
        T_accum = T_inc @ T_accum

        # update source points
        c, s = np.cos(dtheta), np.sin(dtheta)
        R = np.array([[c, -s], [s, c]])
        src = (R @ src.T).T + np.array([dx, dy])

        if abs(dx) < conv_thresh and abs(dy) < conv_thresh and abs(dtheta) < conv_thresh:
            break

    return T_accum, True


# ============================================================================
# Node
# ============================================================================

class OdomIcpNode(Node):
    """Point-to-Line ICP odometry node for STERILINK model car."""

    def __init__(self):
        super().__init__('odom_icp')

        self.declare_parameters(
            namespace='',
            parameters=[
                # --- identity ---
                ('car_id', DEFAULT_CAR_ID, ParameterDescriptor(
                    description="Model car ID.  -1 → use ROS_DOMAIN_ID.")),
                ('publish_tf', True, ParameterDescriptor(
                    description="Broadcast TF odom → <car_id>/base_link.")),
                ('base_frame', 'base_link', ParameterDescriptor(
                    description="Base link suffix (auto-prefixed by car_id).")),
                ('odom_frame', 'odom', ParameterDescriptor(
                    description="Odom frame id.")),

                # --- YDLIDAR G4 hardware ---
                # angle_offset = 0 because car_description TF already handles
                # the 180° backward mounting of the LiDAR.
                ('angle_offset', 0.0, ParameterDescriptor(
                    description="Added to every beam angle. 0 because "
                                "car_description TF handles the 180 deg mounting.")),
                ('range_min_clip', 0.26, ParameterDescriptor(
                    description="Reject ranges below this (m).  0.26 filters robot body.")),
                ('range_max_clip', 12.0, ParameterDescriptor(
                    description="Reject ranges above this (m).  G4 max useful ~ 12 m.")),
                ('zero_range_thresh', 0.05, ParameterDescriptor(
                    description="YDLIDAR reports 0.0 for clear sky; reject <= this.")),

                # --- ICP tuning ---
                ('max_iterations',          50),
                ('convergence_threshold',   1e-4),
                ('max_correspondence_dist', 0.2, ParameterDescriptor(
                    description="Max distance for ICP correspondence (m). "
                                "Tighter = less noise, 0.2 good for model city.")),
                ('min_points',              50),
                ('normal_k',                5, ParameterDescriptor(
                    description="Number of neighbours for surface normal estimation.")),

                # --- motion gate (below LiDAR noise floor) ---
                ('min_translation', 0.015, ParameterDescriptor(
                    description="Ignore displacement below this (m).  15 mm.")),
                ('min_rotation', 0.0175, ParameterDescriptor(
                    description="Ignore rotation below this (rad).  ~1 deg.")),

                # --- jump guard (featureless corridor protection) ---
                ('max_jump_dist', 2.0, ParameterDescriptor(
                    description="Reject ICP step with translation > this (m).")),
                ('max_jump_rot', 0.785, ParameterDescriptor(
                    description="Reject ICP step with rotation > this (rad). ~45 deg.")),
            ]
        )

        # --- resolve car_id ---
        car_id = self.get_parameter('car_id').get_parameter_value().integer_value
        if car_id == -1:
            try:
                car_id = int(os.environ['ROS_DOMAIN_ID'])
            except (KeyError, ValueError):
                self.get_logger().error(
                    "car_id unresolved.  Set '-p car_id:=<int>' or "
                    "export ROS_DOMAIN_ID."
                )
                raise RuntimeError("car_id unresolved")
        self.car_id = car_id

        gp = lambda n: self.get_parameter(n).value
        self.publish_tf       = gp('publish_tf')
        self.odom_frame       = gp('odom_frame')
        self.base_frame       = f"{self.car_id}/{gp('base_frame')}"

        # hardware
        self.angle_offset     = gp('angle_offset')
        self.range_min        = gp('range_min_clip')
        self.range_max        = gp('range_max_clip')
        self.zero_thresh      = gp('zero_range_thresh')

        # ICP
        self.max_iter         = gp('max_iterations')
        self.conv_thresh      = gp('convergence_threshold')
        self.max_corr_dist    = gp('max_correspondence_dist')
        self.min_points       = gp('min_points')
        self.normal_k         = gp('normal_k')

        # gates
        self.min_trans        = gp('min_translation')
        self.min_rot          = gp('min_rotation')
        self.max_jump_dist    = gp('max_jump_dist')
        self.max_jump_rot     = gp('max_jump_rot')

        # --- state ---
        self.prev_scan  = None
        self.prev_stamp = None
        self.T_world    = np.eye(3, dtype=np.float64)
        self.jump_count = 0

        # --- QoS ---
        lidar_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        odom_topic = f'/modelcar{self.car_id}/odom_icp'
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, lidar_qos)
        self.odom_pub = self.create_publisher(
            Odometry, odom_topic, reliable_qos)
        self.tf_broadcaster = (
            tf2_ros.TransformBroadcaster(self) if self.publish_tf else None
        )

        self.get_logger().info(
            f'odom_icp ready  (car_id={self.car_id})\n'
            f'  topic       : {odom_topic}\n'
            f'  frames      : {self.odom_frame} -> {self.base_frame}\n'
            f'  publish_tf  : {self.publish_tf}\n'
            f'  ICP method  : Point-to-Line\n'
            f'  angle_offset: {math.degrees(self.angle_offset):.0f} deg\n'
            f'  range clip  : [{self.range_min:.2f}, {self.range_max:.1f}] m  '
            f'(zero <= {self.zero_thresh})\n'
            f'  corr dist   : {self.max_corr_dist:.2f} m\n'
            f'  motion gate : {self.min_trans*1000:.0f} mm / '
            f'{math.degrees(self.min_rot):.1f} deg\n'
            f'  jump guard  : {self.max_jump_dist:.1f} m / '
            f'{math.degrees(self.max_jump_rot):.0f} deg'
        )

    # ------------------------------------------------------------------
    # scan preprocessing
    # ------------------------------------------------------------------

    def _scan_to_points(self, msg: LaserScan) -> np.ndarray:
        n = len(msg.ranges)
        ranges = np.array(msg.ranges, dtype=np.float64)

        # YDLIDAR zero-range bug
        valid = ranges > self.zero_thresh

        # Robot body filter + sensor spec clip
        lo = max(msg.range_min, self.range_min)
        hi = min(msg.range_max, self.range_max)
        valid &= np.isfinite(ranges) & (ranges > lo) & (ranges < hi)

        # Angles (angle_offset=0; car_description TF handles LiDAR mounting)
        angles = (msg.angle_min
                  + np.arange(n, dtype=np.float64) * msg.angle_increment
                  + self.angle_offset)

        r, a = ranges[valid], angles[valid]
        return np.column_stack([r * np.cos(a), r * np.sin(a)])

    # ------------------------------------------------------------------
    # main callback
    # ------------------------------------------------------------------

    def scan_callback(self, msg: LaserScan):
        pts = self._scan_to_points(msg)
        if len(pts) < self.min_points:
            return

        if self.prev_scan is None:
            self.prev_scan  = pts
            self.prev_stamp = msg.header.stamp
            self.get_logger().info(
                f'first scan accepted  ({len(pts)} pts)')
            return

        T_rel, ok = icp_point_to_line(
            pts, self.prev_scan,
            self.max_iter, self.conv_thresh,
            self.max_corr_dist, self.min_points,
            self.normal_k,
        )
        if not ok:
            self.get_logger().warn('ICP did not converge — skipping')
            self.prev_scan  = pts
            self.prev_stamp = msg.header.stamp
            return

        # T_rel maps current scan → previous scan  (= robot displacement)
        dx, dy, dtheta = extract_pose(T_rel)
        dtheta = normalize_angle(dtheta)
        step_dist = math.hypot(dx, dy)

        # Jump guard
        if step_dist > self.max_jump_dist or abs(dtheta) > self.max_jump_rot:
            self.jump_count += 1
            self.get_logger().warn(
                f'ICP JUMP rejected  '
                f'd={step_dist:.3f} m  dtheta={math.degrees(dtheta):.1f} deg  '
                f'(total rejected: {self.jump_count})')
            self.prev_scan  = pts
            self.prev_stamp = msg.header.stamp
            return

        # dt from sensor timestamps
        t_now  = msg.header.stamp.sec  + msg.header.stamp.nanosec  * 1e-9
        t_prev = self.prev_stamp.sec   + self.prev_stamp.nanosec   * 1e-9
        dt = t_now - t_prev

        # Motion gate
        moved = (step_dist >= self.min_trans) or (abs(dtheta) >= self.min_rot)
        if moved:
            self.T_world = self.T_world @ T_rel
            if dt > 1e-6:
                c, s = np.cos(dtheta), np.sin(dtheta)
                vx_body = ( c * dx + s * dy) / dt
                vy_body = (-s * dx + c * dy) / dt
                vyaw    = dtheta / dt
            else:
                vx_body = vy_body = vyaw = 0.0
        else:
            vx_body = vy_body = vyaw = 0.0

        # keep yaw in [-pi, pi]
        x, y, theta = extract_pose(self.T_world)
        theta = normalize_angle(theta)
        self.T_world = make_T(x, y, theta)

        self.prev_scan  = pts
        self.prev_stamp = msg.header.stamp

        self._publish(msg.header.stamp, x, y, theta, vx_body, vy_body, vyaw)

    # ------------------------------------------------------------------
    # publish odometry + TF
    # ------------------------------------------------------------------

    def _publish(self, stamp, x, y, theta, vx, vy, vyaw):
        quat = yaw_to_quaternion(theta)

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x  = float(x)
        odom.pose.pose.position.y  = float(y)
        odom.pose.pose.orientation = quat

        odom.twist.twist.linear.x  = float(vx)
        odom.twist.twist.linear.y  = float(vy)
        odom.twist.twist.angular.z = float(vyaw)

        pose_cov = [0.0] * 36
        pose_cov[0]  = 0.0100
        pose_cov[7]  = 0.0100
        pose_cov[35] = 0.0004
        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0]  = 0.04
        twist_cov[7]  = 0.04
        twist_cov[35] = 0.02
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp    = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id  = self.base_frame
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.rotation      = quat
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OdomIcpNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
