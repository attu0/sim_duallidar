from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_articubot = FindPackageShare("articubot_one")

    # -------- Gazebo Sim --------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_articubot,
                "launch",
                "launch_sim.launch.py"
            ])
        ),
        launch_arguments={
            "world": PathJoinSubstitution([
                pkg_articubot,
                "worlds",
                "world.world"
            ])
        }.items()
    )

    # -------- Lidar Merger --------
    merger = Node(
        package="dual_lidar_merger",
        executable="laser_merger",
        output="screen"
    )

    # -------- Static TF --------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0","0","0.175",
            "0","0","0",
            "chassis",
            "laser_merged"
        ]
    )

    # -------- RViz --------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([
                pkg_articubot,
                "config",
                "dual_lidar.rviz"
            ])
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        merger,
        static_tf,
        rviz
    ])
