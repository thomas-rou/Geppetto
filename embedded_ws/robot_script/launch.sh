MAIN_BRANCH="develop"
echo "EN ATTENTE" >/tmp/.mission_status
AUDIO_DEVICE_ID=$(pactl list short sinks | grep "USB_PnP_Audio_Device" | awk '{print $1}')
pactl set-default-sink $AUDIO_DEVICE_ID
sudo chown nvidia:nvidia /dev/ttyTHS1 &&
    if [ -d ~/geppetto/ ]; then
        cd ~/geppetto/
        git switch $MAIN_BRANCH
        git pull
    else
        cd ~
        git clone https://github.com/thomas-rou/Geppetto.git
        cd ~/geppetto/
        git switch $MAIN_BRANCH
        git pull
    fi
cd ~/limo_ws
colcon build
source install/setup.bash
cd ~/limo_ws/src/limo_ros2
ros2 launch limo_bringup limo_start.launch.py &
sleep 2
cd ~/geppetto/embedded_ws/ && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=mapper_params_online_async.yaml &
ros2 launch limo_bringup navigation2.launch.py &
sleep 2
cd ~/geppetto/embedded_ws/
colcon build
source install/setup.bash
ros2 launch com_bridge com_bridge_launch.py &
sleep 2
source install/setup.bash
ros2 run rosbridge_server rosbridge_websocket &
sleep 2
source install/setup.bash
ros2 launch explore_lite explore.launch.py &
sleep 1
ros2 topic pub /explore/resume std_msgs/msg/Bool "{data: false}" --once
