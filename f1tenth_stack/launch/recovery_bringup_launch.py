from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    recovery_mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'recovery_mux.yaml'
    )

    # TODO - move the vicon server default to a parameter file, recovery_controller/config/recovery.yaml
    vicon_server_la = DeclareLaunchArgument(
        'vicon_server',
        default_value='172.20.43.22',
        description='IP address of the Vicon VRPN server'
    )

    # Include base bringup (no lidar), overriding mux config with recovery_mux.yaml
    # which adds the ebrake topic at priority 200
    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('f1tenth_stack'),
                'launch',
                'no_lidar_bringup_launch.py'
            )
        ),
        launch_arguments={'mux_config': recovery_mux_config}.items()
    )

    # Vicon pose source — publishes /vrpn_mocap/<rigid_body_name>/pose (PoseStamped)
    vrpn_node = Node(
        package='vrpn_mocap',
        executable='client_node',
        name='client_node',
        namespace='vrpn_mocap',
        parameters=[
            os.path.join(
                get_package_share_directory('vrpn_mocap'),
                'config',
                'client.yaml'
            ),
            {'server': LaunchConfiguration('vicon_server')},
        ]
    )

    # TODO Phase 1: add recovery_node here once recovery_controller package is built

    ld = LaunchDescription([vicon_server_la])
    ld.add_action(base_bringup)
    ld.add_action(vrpn_node)

    return ld
