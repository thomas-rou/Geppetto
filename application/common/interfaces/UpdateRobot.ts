import { RobotCommand } from "../enums/SocketsEvents";
import { BasicCommand } from "./BasicCommand"
import { Position } from "./Position"

export interface UpdateRobot extends BasicCommand {
    command: RobotCommand.UpdateRobot;
    identifier: string;
    status: string;
    position: Position;
}
