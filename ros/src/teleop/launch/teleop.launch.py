from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="teleop",
                executable="teleop",
                name="teleop",
                output="screen",
                parameters=[
                    "config/teleop_config.yaml"
                ],
            ),
        ]
    )
