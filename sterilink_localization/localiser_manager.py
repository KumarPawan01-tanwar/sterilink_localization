#!/usr/bin/env python3
"""
sterilink_localization / localiser_manager  —  STERILINK (LiDAR-only)
======================================================================
Manages the ICP odometry pipeline.  No OptiTrack dependency.

What it does:
    1. Resolves car_id  (param or ROS_DOMAIN_ID)
    2. Publishes a static TF  map → odom  (identity by default,
       so the map overlay and odom arrows share the same origin)
    3. Immediately launches  odom_icp  as a subprocess
    4. Cleans up the subprocess on shutdown

Usage:
    ros2 run sterilink_localization localiser_manager
    ros2 run sterilink_localization localiser_manager --ros-args -p car_id:=7
"""
import os
import subprocess
import signal

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.executors import ExternalShutdownException

DEFAULT_CAR_ID = -1
PKG_NAME = "sterilink_localization"


class LocaliserManager(Node):
    def __init__(self):
        super().__init__('localiser_manager')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('car_id', DEFAULT_CAR_ID, ParameterDescriptor(
                    description="Model car ID.  -1 → use ROS_DOMAIN_ID.")),
                ('publish_tf', True, ParameterDescriptor(
                    description="Let odom_icp broadcast TF odom → <car_id>/base_link.")),
                ('publish_map_to_odom', True, ParameterDescriptor(
                    description="Broadcast a static identity TF map → odom "
                                "so the map overlay is visible in RViz.")),
            ]
        )

        # --- resolve car_id ---
        self.car_id = self.get_parameter('car_id').get_parameter_value().integer_value
        if self.car_id == -1:
            try:
                self.car_id = int(os.environ['ROS_DOMAIN_ID'])
            except (KeyError, ValueError):
                self.get_logger().error(
                    "car_id unresolved.  Set '-p car_id:=<int>' or "
                    "export ROS_DOMAIN_ID."
                )
                raise RuntimeError("car_id unresolved")

        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.publish_map_to_odom = self.get_parameter(
            'publish_map_to_odom').get_parameter_value().bool_value

        # --- static TF:  map → odom  (identity) ---
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        if self.publish_map_to_odom:
            self._publish_static_map_to_odom()

        # --- start odom_icp immediately (no OptiTrack gate) ---
        self.process_icp = None
        self._start_icp()

        self.get_logger().info(
            f'localiser_manager ready  (car_id={self.car_id})\n'
            f'  map → odom TF : {"identity (static)" if self.publish_map_to_odom else "disabled"}\n'
            f'  odom_icp      : launched'
        )

    # ------------------------------------------------------------------
    # static TF
    # ------------------------------------------------------------------

    def _publish_static_map_to_odom(self):
        """Broadcast identity transform  map → odom."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id  = 'odom'
        t.transform.translation = Vector3(x=0.0, y=0.0, z=0.0)
        t.transform.rotation    = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info('Published static TF: map → odom (identity)')

    # ------------------------------------------------------------------
    # subprocess management
    # ------------------------------------------------------------------

    def _start_icp(self):
        """Launch odom_icp as a subprocess."""
        cmd = [
            "ros2", "run", PKG_NAME, "odom_icp",
            "--ros-args",
            "-p", f"car_id:={self.car_id}",
            "-p", f"publish_tf:={self.publish_tf}",
        ]
        self.get_logger().info(f'Starting: {" ".join(cmd)}')
        self.process_icp = subprocess.Popen(
            cmd,
            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL),
        )

    def shutdown_subprocesses(self):
        """Cleanly terminate the ICP subprocess."""
        if self.process_icp and self.process_icp.poll() is None:
            self.get_logger().info('Terminating odom_icp subprocess...')
            self.process_icp.terminate()
            try:
                self.process_icp.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('odom_icp did not exit — sending SIGKILL')
                self.process_icp.kill()
                self.process_icp.wait()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LocaliserManager()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown_subprocesses()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
