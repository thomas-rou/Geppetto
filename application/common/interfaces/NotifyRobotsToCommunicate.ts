import { RobotCommand } from "../enums/SocketsEvents";
import { BasicCommand } from "./BasicCommand"

export interface NotifyRobotsToCommunicate extends BasicCommand {
    command: RobotCommand.InitiateP2P;
}
