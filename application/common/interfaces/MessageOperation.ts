import { Topic, TopicType, Operation, RobotCommand } from "../enums/SocketsEvents";

export interface MessageOperation {
  op: Operation;
  topic: Topic;
  type: TopicType;
  msg?: RobotCommand;
}