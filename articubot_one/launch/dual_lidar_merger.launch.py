from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dual_lidar_merger',
            executable='laser_merger',
            name='laser_merger',
            output='screen'
        )
    ])