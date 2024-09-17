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