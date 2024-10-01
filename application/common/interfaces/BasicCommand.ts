import { RobotCommand } from "../enums/SocketsEvents";

export interface BasicCommand {
    command: RobotCommand;
    timestamp: string;
  }