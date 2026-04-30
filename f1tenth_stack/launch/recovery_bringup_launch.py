from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    recovery_mux_config = os.path.join(
        get_package_share_directory("f1tenth_stack"), "config", "recovery_mux.yaml"
    )

    recovery_config = os.path.join(
        get_package_share_directory("recovery_controller"), "config", "recovery.yaml"
    )

    # Read vicon_server default from recovery.yaml
    with open(recovery_config, "r") as f:
        recovery_params = yaml.safe_load(f)
    vicon_server_default = recovery_params["recovery_node"]["ros__parameters"][
        "vicon_server"
    ]

    vicon_server_la = DeclareLaunchArgument(
        "vicon_server",
        default_value=vicon_server_default,
        description="IP address of the Vicon VRPN server",
    )

    const_v_la = DeclareLaunchArgument(
        "const_v",
        default_value="0.0",
        description="Constant forward velocity (m/s) for joy_teleop human_control. 0.0 = use joystick speed axis.",
    )

    # Include base bringup (no lidar), overriding mux config with recovery_mux.yaml
    # which adds the ebrake topic at priority 200
    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("f1tenth_stack"),
                "launch",
                "no_lidar_bringup_launch.py",
            )
        ),
        launch_arguments={
            "mux_config": recovery_mux_config,
            "const_v": LaunchConfiguration("const_v"),
        }.items(),
    )

    # Vicon pose source — publishes /vrpn_mocap/<rigid_body_name>/pose (PoseStamped)
    vrpn_node = Node(
        package="vrpn_mocap",
        executable="client_node",
        name="client_node",
        namespace="vrpn_mocap",
        parameters=[
            os.path.join(
                get_package_share_directory("vrpn_mocap"), "config", "client.yaml"
            ),
            {"server": LaunchConfiguration("vicon_server")},
        ],
    )

    vesc_config = os.path.join(
        get_package_share_directory("f1tenth_stack"), "config", "vesc.yaml"
    )

    recovery_node = Node(
        package="recovery_controller",
        executable="recovery_node",
        name="recovery_node",
        parameters=[vesc_config, recovery_config],
    )

    ld = LaunchDescription([vicon_server_la, const_v_la])
    ld.add_action(base_bringup)
    ld.add_action(vrpn_node)
    ld.add_action(recovery_node)

    return ld
