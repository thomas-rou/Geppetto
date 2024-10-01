# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# TODO: Split functions and classes into separate files


class Pose:
    def __init__(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


max_width = 3
max_height = 3

robot_count = 2
n_obstacles = 3

# fmt: off
wall_poses = [
    Pose(x=0,    y=1.5,  z=0.1, roll=0, pitch=0, yaw=0  ),  # north wall
    Pose(x=0,    y=-1.5, z=0.1, roll=0, pitch=0, yaw=0  ),  # south wall
    Pose(x=1.5,  y=0,    z=0.1, roll=0, pitch=0, yaw=1.5708),  # east wall
    Pose(x=-1.5, y=0,    z=0.1, roll=0, pitch=0, yaw=1.5708),  # west wall
]
# fmt: on


def load_model_sdf(package_name, model_name, index):
    pkg_project_description = get_package_share_directory(package_name)
    sdf_file = os.path.join(pkg_project_description, "models", model_name, "model.sdf")

    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    # Replace template {index} with the current robot index
    robot_desc = robot_desc.replace("{index}", str(index))

    return robot_desc


def create_robot_state_publisher(robot_desc, index):
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=f"robot_state_publisher{index}",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
        remappings=[("/robot_description", f"/robot_description{index}")],
    )


# fmt: off
def create_spawn_entity(index: int, pose: Pose = Pose()):
    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", f"/robot_description{index}",
            "-name", f"limo_diff_drive{index}",
            "-x", str(pose.x),
            "-y", str(pose.y + index),
            "-z", str(pose.z),
            "-R", str(pose.roll),
            "-P", str(pose.pitch),
            "-Y", str(pose.yaw),
        ],
    )


def create_spawn_obstacle(obstacle_name: str, index: int, pose: Pose) -> Node:
    pkg_project_description = get_package_share_directory("ros_gz_example_description")
    sdf_file = os.path.join(
        pkg_project_description, "models", obstacle_name, "model.sdf"
    )
    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", f"obstacle{index}",
            "-file", sdf_file,
            "-x", str(pose.x),
            "-y", str(pose.y),
            "-z", str(pose.z),
            "-R", str(pose.roll),
            "-P", str(pose.pitch),
            "-Y", str(pose.yaw),
        ],
    )


def get_random_coordinates():
    width  = max_width
    height = max_height

    half_width  = width  / 2
    half_height = height / 2

    random_x = random.uniform(-half_width, half_width)
    random_y = random.uniform(-half_height, half_height)

    return random_x, random_y
# fmt: on


def generate_launch_description():

    # Dynamically generate independent robots

    # Configure ROS nodes for launch

    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_example_gazebo"),
                    "worlds",
                    "diff_drive.sdf",
                ]
            )
        }.items(),
    )

    # Load and setup robots
    robot_state_publishers = []
    spawn_entities = []

    # Load the limo models
    for i in range(robot_count):
        robot_desc = load_model_sdf(
            "ros_gz_example_description", "limo_diff_drive_template", i
        )
        robot_state_publishers.append(create_robot_state_publisher(robot_desc, i))
        spawn_entities.append(create_spawn_entity(i))

    # Spawn random obstacles
    # for i in range(n_obstacles):
    #     spawn_entities.append(create_spawn_obstacle(i, *get_random_coordinates(), z=0.2))

    # Spawn walls
    for i, wall_pose in enumerate(wall_poses):
        spawn_entities.append(create_spawn_obstacle("wall", i, wall_pose))

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    get_package_share_directory("ros_gz_example_bringup"),
                    "config",
                    "ros_gz_example_bridge.yaml",
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_sim,
            bridge,
            *robot_state_publishers,
            *spawn_entities,
        ]
    )
