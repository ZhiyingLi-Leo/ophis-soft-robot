import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("propulsion_pcb_control")
    config_file = os.path.join(package_share, "config", "hardware.yaml")

    serial_port = LaunchConfiguration("serial_port")
    dry_run = LaunchConfiguration("dry_run")
    axis_index = LaunchConfiguration("axis_index")
    stop_button_index = LaunchConfiguration("stop_button_index")

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument(
            "dry_run", default_value="true",
            description="true 时只验证 ROS 2 手柄链路，不打开 PCB 串口"),
        DeclareLaunchArgument("axis_index", default_value="3"),
        DeclareLaunchArgument("stop_button_index", default_value="6"),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{"deadzone": 0.05, "autorepeat_rate": 20.0}],
        ),
        Node(
            package="propulsion_pcb_control",
            executable="propulsion_node",
            name="propulsion_pcb_controller",
            output="screen",
            parameters=[
                config_file,
                {
                    "serial_port": serial_port,
                    "dry_run": ParameterValue(dry_run, value_type=bool),
                    "axis_index": ParameterValue(axis_index, value_type=int),
                    "stop_button_index": ParameterValue(stop_button_index, value_type=int),
                },
            ],
        ),
    ])
