import { RobotCommand } from "../enums/SocketsEvents";
import {BasicCommand} from "./BasicCommand"
import {Position} from "./Position"

export interface StartMission extends BasicCommand {
    command: RobotCommand.StartMission;
    target: string;
    mission_details: {
        orientation: number;
        position: Position
    };
}
