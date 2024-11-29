import os
from com_bridge.common_enums import RobotID, RobotName

def set_mission_status(mission_status):
    with open('/tmp/.mission_status', 'w') as f:
        f.write(mission_status)

def get_mission_status():
    with open('/tmp/.mission_status', 'r') as f:
        return f.read()

def get_robot_id():
    env_robot = os.getenv('ROBOT')
    robot_id = ""
    if env_robot is not None and env_robot[-1] in [RobotID.ROBOT_1, RobotID.ROBOT_2]:
        robot_id = env_robot[-1]
    else:
        robot_id = env_robot
    return robot_id

def get_robot_name():
    env_robot_name = os.getenv('ROBOT')
    if env_robot_name:
        return env_robot_name
    else:
        return RobotName.GAZEBO

def clear_logs():
    with open('/tmp/mission.log', 'w') as f:
        f.write('')
