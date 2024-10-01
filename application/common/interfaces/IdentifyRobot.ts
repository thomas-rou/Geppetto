import { RobotId } from "./../enums/SocketsEvents";

export interface IdentifyRobot {
    command: string;
    target: RobotId;
}