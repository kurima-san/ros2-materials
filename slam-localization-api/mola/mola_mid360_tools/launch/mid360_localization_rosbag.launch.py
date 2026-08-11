from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mola_share = get_package_share_directory('mola_lidar_odometry')
    mola_launch = os.path.join(mola_share, 'launch', 'ros2-lidar-odometry.launch.py')
    if not os.path.exists(mola_launch):
        mola_launch = os.path.join(mola_share, 'ros2-launchs', 'ros2-lidar-odometry.launch.py')

    map_file = LaunchConfiguration('map_file')
    lidar_topic = LaunchConfiguration('lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    base_frame = LaunchConfiguration('base_frame')
    use_rviz = LaunchConfiguration('use_rviz')
    publish_world_map_tf = LaunchConfiguration('publish_world_map_tf')

    return LaunchDescription([
        DeclareLaunchArgument('map_file', description='Absolute path to localization .mm map'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('base_frame', default_value='livox_frame'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument('publish_world_map_tf', default_value='True'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            condition=IfCondition(publish_world_map_tf),
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mola_launch),
            launch_arguments={
                'start_active': 'False',
                'start_mapping_enabled': 'False',
                'use_imu_for_lio': 'True',
                'lidar_topic_name': lidar_topic,
                'imu_topic_name': imu_topic,
                'mola_tf_base_link': base_frame,
                'ignore_lidar_pose_from_tf': 'True',
                'ignore_imu_pose_from_tf': 'True',
                'publish_localization_following_rep105': 'False',
                'mola_initial_map_mm_file': map_file,
                'use_sim_time': 'True',
                'use_rviz': use_rviz,
            }.items(),
        ),
    ])
