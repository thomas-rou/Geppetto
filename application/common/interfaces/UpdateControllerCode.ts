import { RobotCommand } from "../enums/SocketsEvents";
import { BasicCommand } from "./BasicCommand"

export interface UpdateControllerCode extends BasicCommand {
    command: RobotCommand.UpdateControllerCode;
    code: string;
}
