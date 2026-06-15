import os
import launch_ros

from launch import LaunchDescription, LaunchContext
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


# LIDAR OFFSET FOR EACH MODEL CAR
# - z-axis of the laser frame points upward and positive yaw direction is CCW arount the z-axis.
# - the yaw starts at the opposite side of the sensor cable point.
YAW_OFFSET_IN_DEG = {
    #CAR_ID: YAW_OFFSET
    "4":"-45",
    "5":"-45",
    "6":"-45",
    "7":"-60",
    "8":"-45",
    "10": "-45",
    "11": "45",
}
DEFAULT_YAW_OFFSET = '0'

def get_robot_state_publisher_node(context:LaunchContext, lcfg_:LaunchConfiguration):
    """Access declared launch argument 'car_id', append '/' at the end, and return robot_state_publisher_node."""
    car_id_list=context.perform_substitution(lcfg_).split(',')
    nodes = []
    for car_id_ in car_id_list:
        try: lidar_yaw_offset = YAW_OFFSET_IN_DEG[car_id_]
        except: lidar_yaw_offset = DEFAULT_YAW_OFFSET
        node = launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model'), 
                                                              ' visual_only:=', LaunchConfiguration('visual_only'),
                                                              ' lidar_G4_yawoff_deg:=', lidar_yaw_offset
                                                              ]),value_type=str)},
                {'frame_prefix': car_id_ + '/'}
            ]
        )
        nodes.append(node)
    return nodes #return type Iterable[LaunchDescriptionEntity]


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='car_description').find('car_description')
    ego_car_id = os.environ["ROS_DOMAIN_ID"]  #assume ROS_DOMAIN_ID to be ego car ID
    default_model_path = os.path.join(pkg_share, 'urdf/car_description.urdf')

    # Modifying launch arguments declared with DeclareLaunchArgument requires LaunchContext.
    # An OpaqueFunction is used here in order to 
    # - access the declared launch argument 'car_id', 
    # - append '/' at the end, and
    # - return a LaunchDescriptionEntity required by LaunchDescription
    robot_state_publisher_node = OpaqueFunction(function=get_robot_state_publisher_node, args=[LaunchConfiguration('car_id')])
    tf2_laser = launch_ros.actions.Node(package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0', '0', '0','0', '0', '0', '1', ego_car_id+'/laser_frame','laser_frame'],
            )
    tf2_camera = launch_ros.actions.Node(package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0', '0', '0','0', '0', '0', '1', ego_car_id+'/camera_link','camera_link'],
            )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        DeclareLaunchArgument(name='visual_only', default_value='true', description='if true, then wheel link to body is static'),
        DeclareLaunchArgument(name='car_id', default_value=ego_car_id, description='The car ID based on pose_modelcars. By default, value is set to ROS_DOMAIN_ID'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        tf2_laser,
        tf2_camera
    ])
