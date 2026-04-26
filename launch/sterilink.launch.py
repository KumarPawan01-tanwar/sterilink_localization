import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    car_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('car_description'),
                'launch', 'publish_model.launch.py'
            )
        )
    )

    # Static TF: odom → map (identity, since EKF outputs map frame directly)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map',
                   '--child-frame-id', 'odom']
    )

    return LaunchDescription([
        car_description_launch,
        static_tf_map_odom,
        Node(
            package='sterilink_localization',
            executable='encoder_odometry',
            name='encoder_odometry',
            output='screen',
            parameters=[{
                'encoder_topic': '/ackermann_drive_feedback',
                'publish_rate':  50.0,
            }]
        ),
        Node(
            package='sterilink_localization',
            executable='icp_scan_matcher',
            name='icp_scan_matcher',
            output='screen',
            parameters=[{
                'max_iterations':          50,
                'convergence_threshold':   0.0001,
                'max_correspondence_dist': 0.5,
                'min_points':              50,
                'range_min_clip':          0.10,
                'range_max_clip':          20.0,
                'min_translation':         0.015,
                'min_rotation':            0.0175,
            }]
        ),
        Node(
            package='sterilink_localization',
            executable='ekf_localization',
            name='ekf_localization',
            output='screen',
            parameters=[{
                'sigma_v':            0.05,
                'sigma_omega':        0.017,
                'R_icp_x':            0.0100,
                'R_icp_y':            0.0004,
                'R_icp_theta':        0.0004,
                'R_enc_v':            0.0025,
                'innovation_gate':    3.0,
                'max_dt':             0.50,
                'zero_vel_threshold': 0.05,
                'zero_vel_count':     5,
            }]
        ),
    ])
