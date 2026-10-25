from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_pkg',
            executable='pcp',
            name='pcp'
        ),
        Node(
            package='lidar_pkg',
            executable='icp',
            name='icp'
        ),
        Node(
            package='lidar_pkg',
            executable='frame_f',
            name='fa'
        ),
        
    ])