import { RobotCommand } from "@common/enums/RobotCommand";
import { BasicCommand } from "@common/interfaces/BasicCommand"
import { Position } from "@common/types/Position"
import { RobotId } from '@common/enums/RobotId'

export interface StartMission extends BasicCommand {
    command: RobotCommand.StartMission;
    target: RobotId[];
    mission_details: {
        orientation1: number;
        position1: Position;
        orientation2: number;
        position2: Position;
    };
}
