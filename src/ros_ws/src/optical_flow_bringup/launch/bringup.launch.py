import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch_ros.actions import Node
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    tgt_system = LaunchConfiguration("tgt_system")
    tgt_component = LaunchConfiguration("tgt_component")
    log_output = LaunchConfiguration("log_output")
    fcu_protocol = LaunchConfiguration("fcu_protocol")
    respawn_nodes = LaunchConfiguration("respawn_nodes")
    namespace = LaunchConfiguration("namespace")
    use_optical_flow = LaunchConfiguration("use_optical_flow")
    use_optical_flow_aggregator = LaunchConfiguration("use_optical_flow_aggregator")
    optical_flow_ns = LaunchConfiguration("optical_flow_ns")

    declare_fcu_url = DeclareLaunchArgument(
        "fcu_url", default_value="/dev/ttyACM0:57600", description="FCU URL"
    )
    declare_gcs_url = DeclareLaunchArgument(
        "gcs_url", default_value="", description="GCS URL"
    )
    declare_tgt_system = DeclareLaunchArgument(
        "tgt_system", default_value="1", description="Target system ID"
    )
    declare_tgt_component = DeclareLaunchArgument(
        "tgt_component", default_value="1", description="Target component ID"
    )
    declare_log_output = DeclareLaunchArgument(
        "log_output", default_value="both", description="Log output"
    )
    declare_fcu_protocol = DeclareLaunchArgument(
        "fcu_protocol", default_value="v2.0", description="FCU protocol version"
    )
    declare_respawn_nodes = DeclareLaunchArgument(
        "respawn_nodes", default_value="True", description="Respawn Nodes"
    )
    declare_mavros_ns = DeclareLaunchArgument(
        "namespace", default_value="mavros", description="Namespace for MAVROS"
    )
    declare_use_optical_flow = DeclareLaunchArgument(
        "use_optical_flow", default_value="True", description="Use optical_flow"
    )
    declare_use_optical_flow_aggregator = DeclareLaunchArgument(
        "use_optical_flow_aggregator",
        default_value="True",
        description="Use optical_flow_aggregator",
    )
    declare_optical_flow_ns = DeclareLaunchArgument(
        "optical_flow_ns",
        default_value="rpi1",
        description="Namespace for optical flow node",
    )

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace=namespace,
        output=log_output,
        parameters=[
            {"fcu_url": fcu_url},
            {"gcs_url": gcs_url},
            {"tgt_system": tgt_system},
            {"tgt_component": tgt_component},
            {"fcu_protocol": fcu_protocol},
            {"plugin_denylist": ["*"]},
            {
                "plugin_allowlist": [
                    "sys_*",
                    "setpoint_position",
                    "imu",
                    "command",
                    "local_position",
                    "vision_pose",
                    "velocity_observer",
                ]
            },
        ],
        respawn=respawn_nodes,
    )

    optical_flow_node = Node(
        condition=IfCondition(use_optical_flow),
        name="optical_flow_node",
        package="optical_flow",
        executable="optical_flow_node",
        namespace=optical_flow_ns,
        output=log_output,
        respawn=respawn_nodes,
    )

    optical_flow_aggregator_node = Node(
        condition=IfCondition(use_optical_flow_aggregator),
        name="flow_aggregator_node",
        package="optical_flow",
        executable="flow_aggregator_node",
        namespace="",
        output="screen",
        respawn=respawn_nodes,
    )

    return LaunchDescription(
        [
            declare_fcu_url,
            declare_gcs_url,
            declare_tgt_system,
            declare_tgt_component,
            declare_log_output,
            declare_fcu_protocol,
            declare_respawn_nodes,
            declare_mavros_ns,
            declare_use_optical_flow,
            declare_use_optical_flow_aggregator,
            declare_optical_flow_ns,
            mavros_node,
            optical_flow_node,
            optical_flow_aggregator_node,
        ]
    )
