from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="turtlebot_commander",
                executable="turtlebot_commander",
                name="turtlebot_commander",
                output="screen",
            )
        ]
    )
