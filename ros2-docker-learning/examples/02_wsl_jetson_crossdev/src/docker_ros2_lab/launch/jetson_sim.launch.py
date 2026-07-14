from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='docker_ros2_lab', executable='platform_info', output='screen'),
        Node(package='docker_ros2_lab', executable='fake_lidar', output='screen'),
        Node(package='docker_ros2_lab', executable='scan_monitor', output='screen'),
    ])
