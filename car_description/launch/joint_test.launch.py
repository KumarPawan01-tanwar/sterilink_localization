import launch
import launch_ros
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='car_description').find('car_description')
    default_model_path = os.path.join(pkg_share, 'urdf/car_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/car_model_view.rviz')
    default_car_id = os.environ["ROS_DOMAIN_ID"]

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    # This will launch visualize_model.launch.py
    launch_visualize_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share,'launch','visualize_model.launch.py')]),
        launch_arguments={'visual_only': 'false',
                          'model': LaunchConfiguration('model'),
                          'rvizconfig': LaunchConfiguration('rvizconfig'),
                          'car_id': LaunchConfiguration('car_id')
                          }.items()
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='car_id', default_value=default_car_id, description='The car ID based on pose_modelcars. By default, value is set to ROS_DOMAIN_ID'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        launch_visualize_model,
    ])