import os
import launch_ros
from launch import LaunchDescription

def generate_launch_description():

    # Get car ID from environment (e.g. "7")
    ego_car_id = os.environ.get("ROS_DOMAIN_ID", "7")

    # Path to ICP parameters file
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='ad_localiser'
    ).find('ad_localiser')
    icp_param = os.path.join(pkg_share, 'params', 'ICP.yaml')

    # ICP scan matcher node — reads /scan, publishes /odom
    icp_node = launch_ros.actions.Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        parameters=[icp_param]
    )

    # CRITICAL bridge: connects Jetson's "base_link" frame 
    # to STERILINK's "7/base_link" frame
    # Without this, ICP odometry cannot be tracked in RViz
    tf2_base = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', ego_car_id + '/base_link',
            '--child-frame-id', 'base_link'
        ],
    )

    return LaunchDescription([
        icp_node,
        tf2_base,   # always starts together with ICP
    ])
