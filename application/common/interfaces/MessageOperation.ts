import { Topic } from "@common/enums/Topic";
import { TopicType } from "@common/enums/TopicType";
import { Operation } from "@common/enums/Operation";
import { RobotCommand } from "@common/enums/RobotCommand";
export interface MessageOperation {
  op: Operation;
  topic: Topic;
  type: TopicType;
  msg?: RobotCommand;
}