#!/usr/bin/env python3
"""ROS 2 laser scan matcher: keyframe PL-ICP odometry (CSM-free)."""

import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

import icp

DEFAULTS = {
    'publish_odom': 'odom', 'publish_tf': True,
    'base_frame': 'base_link', 'odom_frame': 'odom', 'laser_frame': 'laser',
    'kf_dist_linear': 0.10, 'kf_dist_angular': math.radians(10.0),
    'max_iterations': 20, 'epsilon_xy': 1e-6, 'epsilon_theta': 1e-6,
    'max_correspondence_dist': 0.3, 'outliers_max_perc': 0.9,
    'use_point_to_line_distance': True, 'max_valid_error': 0.1,
}


def yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class LaserScanMatcher(Node):

    def __init__(self):
        super().__init__('laser_scan_matcher')
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        self.p = {n: self.get_parameter(n).value for n in DEFAULTS}

        self.f2b = np.eye(3)          # odom -> base
        self.f2b_kf = np.eye(3)       # odom -> base at keyframe
        self.prev_f2b = np.eye(3)
        self.base_to_laser = np.eye(3)
        self.laser_to_base = np.eye(3)
        self.keyframe = None
        self.last_time = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.p['publish_tf'] else None
        self.odom_pub = (self.create_publisher(Odometry, self.p['publish_odom'], 10)
                         if self.p['publish_odom'] else None)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def lookup_base_to_laser(self, frame):
        try:
            m = self.tf_buffer.lookup_transform(self.p['base_frame'], frame, Time(),
                                                timeout=Duration(seconds=1.0)).transform
            yaw = math.atan2(2 * (m.rotation.w * m.rotation.z + m.rotation.x * m.rotation.y),
                             1 - 2 * (m.rotation.y ** 2 + m.rotation.z ** 2))
            self.base_to_laser = icp.transform_matrix(m.translation.x, m.translation.y, yaw)
            self.laser_to_base = np.linalg.inv(self.base_to_laser)
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'No base->laser tf ({ex}), assuming identity')

    @staticmethod
    def scan_to_points(scan):
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
        return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])

    def on_scan(self, scan):
        points = self.scan_to_points(scan)
        if self.keyframe is None:
            self.lookup_base_to_laser(scan.header.frame_id or self.p['laser_frame'])
            self.keyframe = points
            self.last_time = Time.from_msg(scan.header.stamp)
            return
        if len(points) < 10:
            return

        # first guess: predicted motion since keyframe, in the laser frame
        pr_ch = self.f2b @ np.linalg.inv(self.f2b_kf)
        guess = self.laser_to_base @ np.linalg.inv(self.f2b) @ pr_ch @ self.f2b @ self.base_to_laser

        T, err, _ = icp.icp(points, self.keyframe, init_pose=guess,
                            max_iterations=self.p['max_iterations'],
                            max_correspondence_dist=self.p['max_correspondence_dist'],
                            outliers_max_perc=self.p['outliers_max_perc'],
                            use_point_to_line=self.p['use_point_to_line_distance'],
                            epsilon_xy=self.p['epsilon_xy'],
                            epsilon_theta=self.p['epsilon_theta'])
        if not np.isfinite(err) or err > self.p['max_valid_error']:
            self.get_logger().warn(f'Scan match rejected (err {err:.3f} m)')
            return

        corr = self.base_to_laser @ T @ self.laser_to_base
        self.f2b = self.f2b_kf @ corr
        self.publish(scan.header.stamp)

        x, y, yaw = icp.transform_params(corr)
        if abs(yaw) > self.p['kf_dist_angular'] or x * x + y * y > self.p['kf_dist_linear'] ** 2:
            self.keyframe, self.f2b_kf = points, self.f2b.copy()

    def publish(self, stamp):
        x, y, yaw = icp.transform_params(self.f2b)
        qx, qy, qz, qw = yaw_quat(yaw)
        t = Time.from_msg(stamp)
        dt = max((t - self.last_time).nanoseconds / 1e9, 1e-6)
        self.last_time = t

        if self.odom_pub:
            msg = Odometry()
            msg.header.stamp, msg.header.frame_id = stamp, self.p['odom_frame']
            msg.child_frame_id = self.p['base_frame']
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = qx, qy, qz, qw
            dx, dy, dth = icp.transform_params(np.linalg.inv(self.prev_f2b) @ self.f2b)
            msg.twist.twist.linear.x = dx / dt
            msg.twist.twist.linear.y = dy / dt
            msg.twist.twist.angular.z = dth / dt
            self.prev_f2b = self.f2b.copy()
            self.odom_pub.publish(msg)

        if self.tf_broadcaster:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id = stamp, self.p['odom_frame']
            tf.child_frame_id = self.p['base_frame']
            tf.transform.translation.x, tf.transform.translation.y = x, y
            (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w) = qx, qy, qz, qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanMatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
