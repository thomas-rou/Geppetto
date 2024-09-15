import { Command } from "../enums/command.enum";

export interface RobotRequest {
  command: Command;
  timestamp: string;
}

export interface StartMissionRequest extends RobotRequest {
  command: Command.StartMission;
  mission_details: {
      orientation: string;
      position: { x: number; y: number };
  };
}

export interface EndMissionRequest extends RobotRequest {
  command: Command.EndMission;
}

export interface UpdateRobotRequest extends RobotRequest {
  command: Command.UpdateRobot;
  name: string;
  status: string;
  position: { x: number; y: number };
}

export interface ReturnToBaseRequest extends RobotRequest {
  command: Command.ReturnToBase;
}

export interface UpdateControllerCodeRequest extends RobotRequest {
  command: Command.UpdateControllerCode;
  code: string;
}

export interface NotifyRobotsToCommunicateRequest extends RobotRequest {
  command: Command.P2P;
}

export interface FindFurthestRobotRequest extends RobotRequest {
  command: Command.FindFurthest;
  relative_point: { x: number; y: number };
}