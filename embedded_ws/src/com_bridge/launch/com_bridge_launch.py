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
            Node(
                package="com_bridge",
                executable="identify_robot", 
                name="identify_robot", 
            ),
            Node(
                package="com_bridge",
                executable="mission_status_manager",
                name="mission_status_manager",
            ),
        ]
    )
