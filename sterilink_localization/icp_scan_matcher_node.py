#!/usr/bin/env python3
"""
STERILINK - ICP Scan Matcher Node v2.0
Subscribes : /scan (sensor_msgs/LaserScan)
Publishes  : /odom_icp (nav_msgs/Odometry)

Motion gate: Only publishes when motion exceeds LiDAR noise floor.
Threshold: 15mm translation OR 1 degree rotation.
"""
import numpy as np
from scipy.spatial import cKDTree
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


def normalize_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def make_T(dx, dy, dtheta):
    c, s = np.cos(dtheta), np.sin(dtheta)
    return np.array([[c,-s,dx],[s,c,dy],[0,0,1]], dtype=np.float64)

def extract_pose(T):
    return T[0,2], T[1,2], np.arctan2(T[1,0], T[0,0])

def icp_2d(source, target, max_iter=50, conv_thresh=1e-4,
           max_corr_dist=0.5, min_points=50):
    src = source.copy()
    T_accum = np.eye(3, dtype=np.float64)
    for _ in range(max_iter):
        tree = cKDTree(target)
        dists, idx = tree.query(src, k=1, workers=-1)
        valid = dists < max_corr_dist
        if valid.sum() < min_points:
            return T_accum, False
        src_m, tgt_m = src[valid], target[idx[valid]]
        mu_s, mu_t = src_m.mean(0), tgt_m.mean(0)
        H = (src_m - mu_s).T @ (tgt_m - mu_t)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1,:] *= -1
            R = Vt.T @ U.T
        t = mu_t - R @ mu_s
        dtheta = np.arctan2(R[1,0], R[0,0])
        T_accum = make_T(t[0], t[1], dtheta) @ T_accum
        src = (R @ src.T).T + t
        if np.linalg.norm(t) < conv_thresh and abs(dtheta) < conv_thresh:
            break
    return T_accum, True


class ICPScanMatcherNode(Node):
    def __init__(self):
        super().__init__('icp_scan_matcher')
        self.declare_parameter('max_iterations',          50)
        self.declare_parameter('convergence_threshold',   1e-4)
        self.declare_parameter('max_correspondence_dist', 0.5)
        self.declare_parameter('min_points',              50)
        self.declare_parameter('range_min_clip',          0.10)
        self.declare_parameter('range_max_clip',          20.0)
        self.declare_parameter('min_translation',         0.015)  # 15mm > LiDAR noise
        self.declare_parameter('min_rotation',            0.0175) # 1 degree

        self.max_iter      = self.get_parameter('max_iterations').value
        self.conv_thresh   = self.get_parameter('convergence_threshold').value
        self.max_corr_dist = self.get_parameter('max_correspondence_dist').value
        self.min_points    = self.get_parameter('min_points').value
        self.range_min    = self.get_parameter('range_min_clip').value
        self.range_max    = self.get_parameter('range_max_clip').value
        self.min_trans    = self.get_parameter('min_translation').value
        self.min_rot      = self.get_parameter('min_rotation').value

        self.prev_scan = None
        self.T_world   = np.eye(3, dtype=np.float64)

        lidar_qos = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        reliable  = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, lidar_qos)
        self.odom_pub = self.create_publisher(
            Odometry, '/odom_icp', reliable)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info(
            f'ICP v2.0 started\n'
            f'  min_translation: {self.min_trans*1000:.0f} mm\n'
            f'  min_rotation   : {np.degrees(self.min_rot):.1f} deg'
        )

    def _scan_to_points(self, msg):
        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n, dtype=np.float64) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float64)
        valid = (np.isfinite(ranges) &
                 (ranges > max(msg.range_min, self.range_min)) &
                 (ranges < min(msg.range_max, self.range_max)))
        r, a = ranges[valid], angles[valid]
        return np.column_stack([r*np.cos(a), r*np.sin(a)])

    def scan_callback(self, msg):
        pts = self._scan_to_points(msg)
        if len(pts) < self.min_points:
            return
        if self.prev_scan is None:
            self.prev_scan = pts
            self.get_logger().info('ICP: first scan — ready')
            return

        T_rel, ok = icp_2d(pts, self.prev_scan,
                           self.max_iter, self.conv_thresh,
                           self.max_corr_dist, self.min_points)
        if not ok:
            self.prev_scan = pts
            return

        T_motion = np.linalg.inv(T_rel)
        dx, dy, dtheta = extract_pose(T_motion)

        # MOTION GATE — reject if below noise floor
        if (np.sqrt(dx**2 + dy**2) < self.min_trans and
                abs(normalize_angle(dtheta)) < self.min_rot):
            self.prev_scan = pts
            return

        self.T_world = self.T_world @ T_motion
        x, y, theta = extract_pose(self.T_world)
        theta = normalize_angle(theta)
        self.T_world = make_T(x, y, theta)
        self.prev_scan = pts
        self._publish(msg.header.stamp, x, y, theta)

    def _publish(self, stamp, x, y, theta):
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id  = '7/base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = float(np.sin(theta/2))
        msg.pose.pose.orientation.w = float(np.cos(theta/2))
        cov = [0.0]*36
        cov[0]=0.0100; cov[7]=0.0004; cov[35]=0.0004
        msg.pose.covariance = cov
        self.odom_pub.publish(msg)
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = 'map'
        t.child_frame_id  = '7/base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = float(np.sin(theta/2))
        t.transform.rotation.w = float(np.cos(theta/2))
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ICPScanMatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
