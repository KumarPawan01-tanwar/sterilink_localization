import os
import launch_ros
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    default_car_id = os.environ["ROS_DOMAIN_ID"]
    pkg_share = launch_ros.substitutions.FindPackageShare(package='car_description').find('car_description')
    default_model_path = os.path.join(pkg_share, 'urdf/car_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/car_model_view.rviz')

    # This will launch visualize_model.launch.py
    launch_publish_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share,'launch','publish_model.launch.py')]),
        launch_arguments={'visual_only': LaunchConfiguration('visual_only'),
                          'model': LaunchConfiguration('model'),
                          'rvizconfig': LaunchConfiguration('rvizconfig'),
                          'car_id': LaunchConfiguration('car_id')
                          }.items()
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='visual_only', default_value='true', description='if true, then wheel link to body is static'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='car_id', default_value=default_car_id, description='The car ID based on pose_modelcars. By default, value is set to ROS_DOMAIN_ID'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        launch_publish_model,
        rviz_node,
    ])