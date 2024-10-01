export enum RobotCommand {
    StartMission = 'start_mission',
    EndMission = 'end_mission',
    IdentifyRobot = 'identify_robot',
    UpdateRobot = 'update',
    ReturnToBase = 'return_to_base',
    UpdateControllerCode = 'update_controller_code',
    InitiateP2P = 'P2P',
    FindFurthestRobot = 'find_furthest'
}

export enum Operation {
    publish = 'publish',
    subscribe = 'subscribe',
}

export enum Topic {
    start_mission = '/start_mission_command',
    stop_mission = '/stop_mission_command',
    identify_command1 = '/robot_1/identify_command',
    identify_command2 = '/robot_2/identify_command',
}

export enum TopicType {
    start_mission = 'common_msgs/msg/StartMission',
    stop_mission = 'common_msgs/msg/StopMission',
    identify_robot = 'common_msgs/msg/IdentifyRobot',
}

export enum RobotList {
    gazebo = '0',
    robot1 = '1',
    robot2 = '2'
}