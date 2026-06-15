#!/usr/bin/env python3
"""
STERILINK - Encoder Odometry Node
Subscribes: /ackermann_drive_feedback (ackermann_msgs/AckermannDrive)
Publishes:  /odom (nav_msgs/Odometry)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__('encoder_odometry')
        self.declare_parameter('encoder_topic', '/ackermann_drive_feedback')
        self.declare_parameter('publish_rate', 50.0)
        enc_topic = self.get_parameter('encoder_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.acc_vel = 0.0
        self.acc_x = 0.0
        self.last_time = None
        best_effort = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        reliable = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self.tick_sub = self.create_subscription(
            AckermannDrive, enc_topic, self._feedback_callback, best_effort)
        self.odom_pub = self.create_publisher(Odometry, '/odom', reliable)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_callback)
        self.get_logger().info(
            f'Encoder Odometry Node started\n'
            f'  topic       : {enc_topic}\n'
            f'  publish_rate: {self.publish_rate} Hz'
        )

    def _feedback_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        if 0.0 < dt < 2.0:
            self.acc_vel = float(msg.speed)
            self.acc_x += self.acc_vel * dt
        self.last_time = now

    def _publish_callback(self):
        stamp = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.acc_x
        msg.pose.pose.orientation.w = 1.0
        cov_p = [0.0] * 36
        cov_p[0] = 1e6
        cov_p[7] = 1e6
        cov_p[35] = 1e6
        msg.pose.covariance = cov_p
        msg.twist.twist.linear.x = self.acc_vel
        cov_t = [0.0] * 36
        cov_t[0] = 0.01
        cov_t[35] = 1e6
        msg.twist.covariance = cov_t
        self.odom_pub.publish(msg)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.acc_x
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
