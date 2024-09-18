cd geppetto/embedded_ws/ & colcon build & source install/setup.bash & ros2 run com_bridge com_serv 
source install/setup.bash & ros2 run rosbridge_server rosbridge_websocket
cd ~/limo_ws/src/limo_ros2 & ros2 launch limo_bringup limo_start.launch.py