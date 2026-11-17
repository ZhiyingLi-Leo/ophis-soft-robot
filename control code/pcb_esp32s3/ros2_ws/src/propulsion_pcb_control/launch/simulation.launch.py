from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="propulsion_pcb_control",
            executable="propulsion_node",
            name="propulsion_pcb_controller",
            output="screen",
            parameters=[{
                "dry_run": True,
                "diagnostic": True,
                "axis_index": 3,
                "stop_button_index": 6,
                "max_speed": 3200,
                "max_acceleration": 3200.0,
            }],
        ),
        Node(
            package="propulsion_pcb_control",
            executable="simulated_joy_node",
            name="simulated_joy",
            output="screen",
            parameters=[{"axis_index": 3, "stop_button_index": 6}],
        ),
    ])
