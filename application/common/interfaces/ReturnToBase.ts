import { RobotCommand } from "../enums/SocketsEvents";
import { BasicCommand } from "./BasicCommand"

export interface ReturnToBase extends BasicCommand {
    command: RobotCommand.ReturnToBase;
}
