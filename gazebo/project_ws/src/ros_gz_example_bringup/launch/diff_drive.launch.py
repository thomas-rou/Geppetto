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

import math
import os
import random
import numpy as np
from abc import ABC, abstractmethod
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# TODO: Split functions and classes into separate files


class Pose:
    def __init__(self, x=0.0, y=0.0, z=0.1, roll=0.0, pitch=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def to_array(self):
        return np.array([self.x, self.y, self.z])


class Size:
    def __init__(self, x=0.0, y=0.1, z=0.2):
        self.x = x
        self.y = y
        self.z = z

    def to_array(self):
        return np.array([self.x, self.y, self.z])


# pure abstract class
class Entity(ABC):
    pose: Pose
    size: Size
    name: str

    @abstractmethod
    def __init__(self):
        pass

    def build_entity(self, pose: Pose = None, size: Size = None):
        # Initialize pose with provided values or default values
        if pose is None:
            self.pose = Pose()
        else:
            self.pose = Pose(
                x=pose.x if hasattr(pose, "x") else 0.0,
                y=pose.y if hasattr(pose, "y") else 0.0,
                z=pose.z if hasattr(pose, "z") else 0.0,
                roll=pose.roll if hasattr(pose, "roll") else 0.0,
                pitch=pose.pitch if hasattr(pose, "pitch") else 0.0,
                yaw=pose.yaw if hasattr(pose, "yaw") else 0.0,
            )

        # Initialize size with provided values or default values
        if size is None:
            self.size = Size()
        else:
            self.size = Size(
                x=size.x if hasattr(size, "x") else 0.0,
                y=size.y if hasattr(size, "y") else 0.0,
                z=size.z if hasattr(size, "z") else 0.0,
            )


# pure abstract class
class Obstacle(Entity):
    _id = 0

    @abstractmethod
    def __init__(self, name: str):
        pass

    def get_id():
        current_id = Obstacle._id
        Obstacle._id += 1
        return current_id


class Robot(Entity):
    def __init__(self, name="robot", pose: Pose = None, size: Size = None):
        self.name = name
        self.build_entity(pose, size)


class Wall(Obstacle):
    def __init__(self, pose: Pose = None, size: Size = None):
        self.name = "wall"
        self.build_entity(pose, size)


max_width = 10
max_height = 5

wall_thickness = 0.1

robot_count = 2
n_wall_obstacles = 10

horizontal_yaw = math.pi / 2

# fmt: off
walls = [
    Wall(pose=Pose(y= max_width/2),               size=Size(x=max_width)), # west wall
    Wall(pose=Pose(y=-max_width/2),               size=Size(x=max_width)), # east wall
    Wall(pose=Pose(x= max_width/2 , yaw=horizontal_yaw),  size=Size(x=max_width)), # north wall
    Wall(pose=Pose(x=-max_width/2 , yaw=horizontal_yaw),  size=Size(x=max_width)), # south wall
]

robots = [
    Robot(name="pino"),
    Robot(name="chio", pose=Pose(y=1)),
]

def check_spawn_kill(obstacle: Obstacle):

    def get_bounding_box(pose, size):
        min_corner = pose.to_array() - size.to_array() / 2.0
        max_corner = pose.to_array() + size.to_array() / 2.0
        return min_corner, max_corner

    combined_entitites = robots + walls

    for entity in combined_entitites:

        entity_min, entity_max = get_bounding_box(entity.pose, entity.size)
        obstacle_min, obstacle_max = get_bounding_box(obstacle.pose, obstacle.size)

        # Check if bounding boxes intersect (AABB collision detection)
        for i in range(3):  # 3 dimensions: x, y, z
            if entity_max[i] >= obstacle_min[i] and entity_min[i] <= obstacle_max[i]:
                return False
    
    return True


def generate_random_wall_obstacles():
    while True:
        start = random.randint(0, 5) # start at any of 4 walls or anywhere
        size=Size(x=random.uniform(0.5, max_width/2))

        match(start):
            case 0 :
                wall = Wall(
                    pose=Pose(y=random.uniform(-max_width / 2 + wall_thickness, max_width / 2 - wall_thickness), x=max_width / 2 - size.x / 2), size=size)
            case 1 :
                wall = Wall(
                    pose=Pose(y=random.uniform(-max_width / 2 + wall_thickness, max_width / 2 - wall_thickness), x=-max_width / 2 + size.x / 2), size=size)
            case 2 :
                wall = Wall(
                    pose=Pose(x=random.uniform(-max_width / 2 + wall_thickness, max_width / 2 - wall_thickness), y=max_width / 2 - size.x / 2, yaw=horizontal_yaw), size=size)
            case 3 :
                wall = Wall(
                    pose=Pose(x=random.uniform(-max_width / 2 + wall_thickness, max_width / 2 - wall_thickness), y=-max_width / 2 + size.x / 2, yaw=horizontal_yaw), size=size)
            case _ :
                wall = Wall(
                    pose=Pose(x=random.uniform(-max_height / 2 + wall_thickness, max_height / 2 - wall_thickness), y=random.uniform(-max_width  / 2 + wall_thickness, max_width / 2 - wall_thickness), yaw=random.uniform(0, math.pi)), size=size)
        
        if not check_spawn_kill(wall):
            break

    walls.append(wall)
    return wall
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
# see if can combine or make class mehtod

def create_spawn_robot(robot: Robot, index: int):
    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", f"/robot_description{index}",
            "-name", f"limo_diff_drive{index}",
            "-x", str(robot.pose.x),
            "-y", str(robot.pose.y),
            "-z", str(robot.pose.z),
            "-R", str(robot.pose.roll),
            "-P", str(robot.pose.pitch),
            "-Y", str(robot.pose.yaw),
        ],
    )


def create_spawn_obstacle(obstacle: Obstacle) -> Node:
    pkg_project_description = get_package_share_directory("ros_gz_example_description")
    sdf_file = os.path.join(
        pkg_project_description, "models", obstacle.name, "model.sdf"
    )

    with open(sdf_file, "r") as infp:
        obstacle_desc = infp.read()

    index = Obstacle.get_id()
    
    obstacle_desc = obstacle_desc.replace("{index}", str(index))
    obstacle_desc = obstacle_desc.replace("{size_x}", str(obstacle.size.x))
    obstacle_desc = obstacle_desc.replace("{size_y}", str(obstacle.size.y))
    obstacle_desc = obstacle_desc.replace("{size_z}", str(obstacle.size.z))

    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", f"{obstacle.name}{index}",
            "-string", obstacle_desc,
            "-x", str(obstacle.pose.x),
            "-y", str(obstacle.pose.y),
            "-z", str(obstacle.pose.z),
            "-R", str(obstacle.pose.roll),
            "-P", str(obstacle.pose.pitch),
            "-Y", str(obstacle.pose.yaw),
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
    for i, robot in enumerate(robots):
        robot_desc = load_model_sdf(
            "ros_gz_example_description", "limo_diff_drive_template", i
        )
        robot_state_publishers.append(create_robot_state_publisher(robot_desc, i))
        spawn_entities.append(create_spawn_robot(robot, i))

    # Spawn random obstacles
    for _ in range(n_wall_obstacles):
        wall = generate_random_wall_obstacles()
        spawn_entities.append(create_spawn_obstacle(wall))

    # Spawn walls
    for wall in walls:
        spawn_entities.append(create_spawn_obstacle(wall))

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
