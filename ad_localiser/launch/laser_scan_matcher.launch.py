import os
import launch_ros
from launch import LaunchDescription

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ad_localiser').find('ad_localiser')
    icp_param = os.path.join(pkg_share,'params','ICP.yaml')

    icp_node = launch_ros.actions.Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        parameters=[icp_param]
    )

    return LaunchDescription([
        icp_node
    ])