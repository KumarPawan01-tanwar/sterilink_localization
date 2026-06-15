#!/usr/bin/env python3
"""
STERILINK - EKF Localization Node v4.0
Implements EXACTLY the design document specification.

State: x = [x, y, theta, v, omega]
Q: Diagonal as per design document Section 6.4
R_icp: Asymmetric corridor noise as per Section 7.1
R_enc: 0.0025 as per Section 7.2
Zero-velocity lock: Freezes ICP updates when stopped
Innovation gate: Mahalanobis chi-squared test
Joseph form covariance update
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


def normalize_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def quat_to_yaw(qx, qy, qz, qw):
    return 2.0 * np.arctan2(qz, qw)

def yaw_to_quat(theta):
    return 0.0, 0.0, float(np.sin(theta/2.0)), float(np.cos(theta/2.0))


class EKFLocalizationNode(Node):
    def __init__(self):
        super().__init__('ekf_localization')

        # Parameters matching design document exactly
        self.declare_parameter('sigma_v',            0.05)   # Section 6.4
        self.declare_parameter('sigma_omega',        0.017)  # Section 6.4
        self.declare_parameter('R_icp_x',            0.0100) # Section 7.1
        self.declare_parameter('R_icp_y',            0.0004) # Section 7.1
        self.declare_parameter('R_icp_theta',        0.0004) # Section 7.1
        self.declare_parameter('R_enc_v',            0.0025) # Section 7.2
        self.declare_parameter('innovation_gate',    3.0)    # chi2(3) at 3-sigma
        self.declare_parameter('max_dt',             0.50)
        self.declare_parameter('zero_vel_threshold', 0.05)   # m/s
        self.declare_parameter('zero_vel_count',     5)      # consecutive readings

        sigma_v      = self.get_parameter('sigma_v').value
        sigma_omega  = self.get_parameter('sigma_omega').value
        self.max_dt  = self.get_parameter('max_dt').value
        self.zero_thresh  = self.get_parameter('zero_vel_threshold').value
        self.zero_count_thresh = int(self.get_parameter('zero_vel_count').value)
        self.innov_gate = self.get_parameter('innovation_gate').value

        # ── State vector x = [x, y, theta, v, omega] ─────────────────────
        self.x = np.zeros(5, dtype=np.float64)

        # ── Initial covariance P0 per Section 9.2 ─────────────────────────
        self.P = np.diag([
            0.0100,   # x:     (0.10m)²
            0.0100,   # y:     (0.10m)²
            0.0025,   # theta: (0.05rad)²
            0.0400,   # v:     (0.20m/s)²
            0.00757,  # omega: (0.087rad/s)²
        ]).astype(np.float64)

        # ── Q: DIAGONAL per design document Section 6.4 ───────────────────
        self.Q = np.diag([
            0.0004,   # sigma²_x
            0.0004,   # sigma²_y
            0.0001,   # sigma²_theta
            sigma_v**2,     # sigma²_v
            sigma_omega**2, # sigma²_omega
        ]).astype(np.float64)

        # ── R matrices per design document ────────────────────────────────
        self.R_icp = np.diag([
            self.get_parameter('R_icp_x').value,
            self.get_parameter('R_icp_y').value,
            self.get_parameter('R_icp_theta').value,
        ]).astype(np.float64)

        self.R_enc = np.array([[self.get_parameter('R_enc_v').value]],
                               dtype=np.float64)

        # ── H matrices (constant) ─────────────────────────────────────────
        self.H_enc = np.array([[0., 0., 0., 1., 0.]], dtype=np.float64)
        self.H_icp = np.zeros((3,5), dtype=np.float64)
        self.H_icp[0,0] = self.H_icp[1,1] = self.H_icp[2,2] = 1.0

        # ── Zero velocity state ───────────────────────────────────────────
        self.zero_count  = 0
        self.is_stopped  = False
        self.last_enc_time = None

        # ── Velocity low-pass filter ──────────────────────────────────────
        self.vel_filtered = 0.0
        self.vel_alpha    = 0.4

        best_effort = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        reliable = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)

        self.enc_sub = self.create_subscription(
            Odometry, '/odom', self.encoder_callback, best_effort)
        self.icp_sub = self.create_subscription(
            Odometry, '/odom_icp', self.icp_callback, reliable)
        self.filtered_pub = self.create_publisher(
            Odometry, '/odometry/filtered', reliable)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info(
            f'EKF v4.0 started — per design document\n'
            f'  Q diagonal: x={self.Q[0,0]}, y={self.Q[1,1]}, '
            f'th={self.Q[2,2]}, v={self.Q[3,3]:.4f}, w={self.Q[4,4]:.4f}\n'
            f'  R_icp: x={self.R_icp[0,0]}, y={self.R_icp[1,1]}, '
            f'th={self.R_icp[2,2]}\n'
            f'  R_enc: {self.R_enc[0,0]}\n'
            f'  zero_vel_threshold: {self.zero_thresh} m/s\n'
            f'  innovation_gate: {self.innov_gate} sigma'
        )

    # ── ENCODER CALLBACK — PREDICT + UPDATE v ─────────────────────────────
    def encoder_callback(self, msg):
        now = self._stamp_to_sec(msg.header.stamp)
        if self.last_enc_time is None:
            self.last_enc_time = now
            return

        dt = now - self.last_enc_time
        self.last_enc_time = now

        if dt <= 0.0 or dt > self.max_dt:
            return

        v_raw = float(msg.twist.twist.linear.x)

        # Low-pass filter on velocity
        self.vel_filtered = (self.vel_alpha * v_raw +
                            (1.0 - self.vel_alpha) * self.vel_filtered)

        # Zero velocity detection
        if abs(self.vel_filtered) < self.zero_thresh:
            self.zero_count = min(self.zero_count + 1, self.zero_count_thresh + 1)
        else:
            self.zero_count = 0

        self.is_stopped = (self.zero_count >= self.zero_count_thresh)

        if self.is_stopped:
            # FREEZE: hold position, force v and omega to zero
            self.x[3] = 0.0
            self.x[4] = 0.0
            self._publish(msg.header.stamp)
            return

        # PREDICT — unicycle model per Section 6.1
        self._predict(dt)

        # UPDATE A — encoder velocity per Section 7.2
        self._update_encoder(self.vel_filtered)

        self._publish(msg.header.stamp)

    # ── ICP CALLBACK — UPDATE B ───────────────────────────────────────────
    def icp_callback(self, msg):
        # ZERO VELOCITY LOCK — reject ALL ICP when stopped
        if self.is_stopped:
            return

        x_icp     = float(msg.pose.pose.position.x)
        y_icp     = float(msg.pose.pose.position.y)
        theta_icp = quat_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        self._update_icp(x_icp, y_icp, theta_icp)
        self._publish(msg.header.stamp)

    # ── PREDICT — Section 6.1 ─────────────────────────────────────────────
    def _predict(self, dt):
        x, y, theta, v, omega = self.x

        # Unicycle motion model
        self.x = np.array([
            x + v * np.cos(theta) * dt,
            y + v * np.sin(theta) * dt,
            normalize_angle(theta + omega * dt),
            v,
            omega,
        ])

        # Jacobian F per Section 6.2
        F = np.eye(5, dtype=np.float64)
        F[0,2] = -v * np.sin(theta) * dt
        F[0,3] =  np.cos(theta) * dt
        F[1,2] =  v * np.cos(theta) * dt
        F[1,3] =  np.sin(theta) * dt
        F[2,4] =  dt

        # Covariance prediction with DIAGONAL Q per Section 6.3 & 6.4
        self.P = F @ self.P @ F.T + self.Q

    # ── UPDATE A — encoder velocity ────────────────────────────────────────
    def _update_encoder(self, v_meas):
        H = self.H_enc
        z = np.array([v_meas])
        innov = z - H @ self.x
        S = H @ self.P @ H.T + self.R_enc
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K.flatten() * innov[0]
        self.x[2] = normalize_angle(self.x[2])
        # Joseph form
        I_KH = np.eye(5) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R_enc @ K.T

    # ── UPDATE B — ICP pose ────────────────────────────────────────────────
    def _update_icp(self, x_icp, y_icp, theta_icp):
        H = self.H_icp
        z = np.array([x_icp, y_icp, theta_icp])
        innov    = z - H @ self.x
        innov[2] = normalize_angle(innov[2])  # MANDATORY angle wrap

        S     = H @ self.P @ H.T + self.R_icp
        S_inv = np.linalg.inv(S)

        # Mahalanobis innovation gate per Section 10
        mahal = float(innov.T @ S_inv @ innov)
        gate_sq = self.innov_gate**2 * 3.0  # chi2(3)
        if mahal > gate_sq:
            self.get_logger().debug(
                f'ICP rejected: mahal={mahal:.2f} > gate={gate_sq:.2f}')
            return

        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ innov
        self.x[2] = normalize_angle(self.x[2])
        # Joseph form — numerically stable
        I_KH = np.eye(5) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R_icp @ K.T

    # ── PUBLISH ────────────────────────────────────────────────────────────
    def _publish(self, stamp):
        x, y, theta, v, omega = self.x
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id  = 'base_link'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(theta)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x  = float(v)
        msg.twist.twist.angular.z = float(omega)
        cov_p = [0.0]*36
        cov_p[0]  = float(self.P[0,0])
        cov_p[7]  = float(self.P[1,1])
        cov_p[35] = float(self.P[2,2])
        msg.pose.covariance = cov_p
        cov_t = [0.0]*36
        cov_t[0]  = float(self.P[3,3])
        cov_t[35] = float(self.P[4,4])
        msg.twist.covariance = cov_t
        self.filtered_pub.publish(msg)
        self._broadcast_tf(stamp, x, y, theta)

    def _broadcast_tf(self, stamp, x, y, theta):
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = 'map'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(theta)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _stamp_to_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
