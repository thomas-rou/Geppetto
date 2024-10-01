import { RobotCommand } from "../enums/SocketsEvents";
import { BasicCommand } from "./BasicCommand"
import { Position } from "./Position"

export interface FindFurthestRobot extends BasicCommand {
    command: RobotCommand.FindFurthestRobot;
    relative_point: Position
    timestamp: string;
}
