from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    slam = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "scan_topic": "/scan",
            "base_frame": "chassis",
            "odom_frame": "odom",
            "map_frame": "map",
            "resolution": 0.05,
            "max_laser_range": 12.0
        }]
    )

    return LaunchDescription([slam])
