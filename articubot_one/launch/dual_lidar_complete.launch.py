import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'articubot_one'
    
    # Path to RViz config
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'dual_lidar.rviz'
    )
    
    # Include Gazebo simulation launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'launch_sim.launch.py'
            )
        ])
    )
    
    # Laser merger node
    laser_merger = Node(
        package='dual_lidar_merger',
        executable='laser_merger',
        name='laser_merger',
        output='screen'
    )
    
    # RViz with config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        laser_merger,
        rviz_node
    ])