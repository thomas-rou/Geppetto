import { RobotCommand } from "../enums/SocketsEvents";
import {BasicCommand} from "./BasicCommand"

export interface EndMission extends BasicCommand {
    command: RobotCommand.EndMission;
    target: string;
}
