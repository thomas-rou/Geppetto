export enum RobotCommandFromInterface {
    StartMission = 'start_mission',
    EndMission = 'end_mission',
    IdentifyRobot = 'identify_robot',
    UpdateRobot = 'update',
    ReturnToBase = 'return_to_base',
    UpdateControllerCode = 'update_controller_code',
    NotifyRobotsToCommunicate = 'initiate_p2p',
    FindFurthestRobot = 'find_furthest'
}

export enum Operation {
    publish = 'publish',
    subscribe = 'subscribe',
}

export enum Topic {
    start_mission = '/start_mission_command',
    stop_mission = '/stop_mission_command',
}

export enum TopicType {
    start_mission = 'common_msgs/msg/StartMission',
    stop_mission = 'common_msgs/msg/StopMission',
}

export enum Command {
    StartMission = 'start_mission',
    EndMission = 'end_mission',
    UpdateRobot = 'update',
    ReturnToBase = 'return_to_base',
    UpdateControllerCode = 'update_controller_code',
    P2P = 'P2P',
    FindFurthest = 'find_furthest',
}