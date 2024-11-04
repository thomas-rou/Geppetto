#!/bin/bash

# Navigate to embedded directory
cd "/home/abdul/Documents/geppetto/embedded_ws/" || exit 1

# Run colcon build
colcon build

# Source the setup script
source install/setup.sh

# Navigate to gazebo directory
cd "/home/abdul/Documents/geppetto/gazebo/project_ws/" || exit 1

# Run the colcon build with testing enabled
colcon build --cmake-args -DBUILD_TESTING=ON

# Source the setup script
source install/setup.sh

# Launch the ROS2 launch file
ros2 launch ros_gz_example_bringup diff_drive.launch.py