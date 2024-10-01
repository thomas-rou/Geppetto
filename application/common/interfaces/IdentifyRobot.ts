import { RobotList } from "./../enums/SocketsEvents";

export interface IdentifyRobot {
    command: string;
    target: RobotList;
}