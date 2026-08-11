import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_localization_ros2')

    default_params = os.path.join(
        pkg_share, 'param', 'mid360_handheld.yaml')
    default_rviz = os.path.join(
        pkg_share, 'param', 'mid360_handheld.rviz')
    base_launch = os.path.join(
        pkg_share, 'launch', 'lidar_localization.launch.py')

    localization_param_dir = LaunchConfiguration('localization_param_dir')
    cloud_topic = LaunchConfiguration('cloud_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    global_frame_id = LaunchConfiguration('global_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    start_rviz = LaunchConfiguration('start_rviz')
    publish_world_map_tf = LaunchConfiguration('publish_world_map_tf')

    # Visualization anchor:
    # lets RViz use "map" before the localizer has accepted /initialpose.
    world_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_tf',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', global_frame_id,
        ],
        condition=IfCondition(publish_world_map_tf),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            'localization_param_dir': localization_param_dir,
            'cloud_topic': cloud_topic,
            'imu_topic': imu_topic,
            'global_frame_id': global_frame_id,
            'base_frame_id': base_frame_id,
            'lidar_frame_id': lidar_frame_id,
            'imu_frame_id': imu_frame_id,
            'publish_lidar_tf': 'false',
            'publish_imu_tf': 'false',
            'use_imu_preintegration': 'true',
            'enable_map_odom_tf': 'false',
            'use_odom': 'false',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'localization_param_dir',
            default_value=default_params,
            description='Parameter YAML for handheld MID-360 localization.'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('global_frame_id', default_value='map'),
        DeclareLaunchArgument('base_frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('lidar_frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('imu_frame_id', default_value='livox_frame'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='false: realtime sensor, true: rosbag playback with --clock.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument(
            'publish_world_map_tf',
            default_value='true',
            description='Publish static world -> map for RViz visualization.'),
        world_to_map_tf,
        localization,
        rviz,
    ])
