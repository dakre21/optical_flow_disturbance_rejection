import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mocap4r2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("mocap4r2_optitrack_driver"),
                    "launch",
                    "optitrack2.launch.py",
                ]
            )
        ),
    )

    mocap_to_mavros_node = Node(
        name="relay_node",
        package="mocap_to_mavros",
        executable="relay_node",
        namespace="",
        output="screen",
    )

    return LaunchDescription(
        [
            mocap4r2_launch,
            mocap_to_mavros_node
        ]
    )
