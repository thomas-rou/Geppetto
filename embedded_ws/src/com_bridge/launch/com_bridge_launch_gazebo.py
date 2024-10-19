from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="com_bridge",
                executable="mission_controller",
                name="mission_controller", 
            ),
        ]
    )
