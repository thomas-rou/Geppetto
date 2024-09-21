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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# TODO: Split functions into seperate files

def load_model_sdf(index):
    pkg_project_description = get_package_share_directory('ros_gz_example_description')
    sdf_file = os.path.join(pkg_project_description, 'models', 'limo_diff_drive_template', 'model.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Replace template {index} with the current robot index
    robot_desc = robot_desc.replace('{index}', str(index))
    
    return robot_desc

def create_robot_state_publisher(robot_desc, index):
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher{index}',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ],
        remappings=[
            ('/robot_description', f'/robot_description{index}')
        ]
    )


def create_spawn_entity(index, x = 0.0, y = 0.0, z = 0.28):
    return Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', f"/robot_description{index}",
            '-name', f'limo_diff_drive{index}',
            '-x', str(x),
            '-y', str(y + index),
            '-z', str(z),
        ]
    )


def generate_launch_description():

    # Dynamicaly generate independant robots
    robot_count = 2

    # Configure ROS nodes for launch

    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            get_package_share_directory('ros_gz_example_gazebo'),
            'worlds',
            'diff_drive.sdf'
        ])}.items(),
    )

    # Load and setup robots

    robot_state_publishers = []
    spawn_entities = []

    # TODO: hard coded pose for now
    for i in range(robot_count):
        robot_desc = load_model_sdf(i)
        robot_state_publishers.append(create_robot_state_publisher(robot_desc, i))
        spawn_entities.append(create_spawn_entity(i))

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory('ros_gz_example_bringup'), 'config', 'ros_gz_example_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        *robot_state_publishers,
        *spawn_entities,
    ])
