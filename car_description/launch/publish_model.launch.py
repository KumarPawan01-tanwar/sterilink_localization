import os
import launch_ros
from launch import LaunchDescription, LaunchContext
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

# URDF visual yaw offset per car — controls how car model looks in RViz
# NOTE: laser_frame TF is owned by ydlidar_launch.py — NOT here
YAW_OFFSET_IN_DEG = {
    "4": "-45", "5": "-45", "6": "-45",
    "7": "135",  # Car 7
    "8": "-45", "10": "-45", "11": "45",
}
DEFAULT_YAW_OFFSET = '0'

def get_robot_state_publisher_node(context: LaunchContext, lcfg_: LaunchConfiguration):
    car_id_list = context.perform_substitution(lcfg_).split(',')
    nodes = []
    for car_id_ in car_id_list:
        try:
            lidar_yaw_offset = YAW_OFFSET_IN_DEG[car_id_]
        except:
            lidar_yaw_offset = DEFAULT_YAW_OFFSET
        node = launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': ParameterValue(
                    Command(['xacro ', LaunchConfiguration('model'),
                             ' visual_only:=', LaunchConfiguration('visual_only'),
                             ' lidar_G4_yawoff_deg:=', lidar_yaw_offset]),
                    value_type=str)},
                {'frame_prefix': car_id_ + '/'}
            ]
        )
        nodes.append(node)
    return nodes

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='car_description').find('car_description')
    ego_car_id = os.environ.get("ROS_DOMAIN_ID", "7")
    default_model_path = os.path.join(pkg_share, 'urdf/car_description.urdf')

    robot_state_publisher_node = OpaqueFunction(
        function=get_robot_state_publisher_node,
        args=[LaunchConfiguration('car_id')]
    )

    # Bridge: connects 7/camera_link to camera_link
    tf2_camera = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_camera',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', ego_car_id + '/camera_link',
            '--child-frame-id', 'camera_link'
        ],
    )

    # Bridge: connects 7/base_link to base_link (CRITICAL for ICP)
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

    # NOTE: base_link → laser_frame is published by ydlidar_launch.py
    # with the correct -45° yaw offset. Do NOT add it here.

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        DeclareLaunchArgument(name='visual_only', default_value='true'),
        DeclareLaunchArgument(name='car_id', default_value=ego_car_id),
        DeclareLaunchArgument(name='model', default_value=default_model_path),
        robot_state_publisher_node,
        tf2_camera,
        tf2_base,
    ])
