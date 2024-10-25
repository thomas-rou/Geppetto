import { Injectable } from '@nestjs/common';
import { RobotStatus } from '@common/interfaces/RobotStatus';
import { LogMessage } from '@common/interfaces/LogMessage';
import { RobotService } from '../robot/robot.service';
import { Topic } from '@common/enums/Topic';
import { TopicType } from '@common/enums/TopicType';
import { RobotId } from '@common/enums/RobotId';
import { MissionCommandGateway } from '@app/gateways/mission-command/mission-command.gateway';
@Injectable()
export class SubscriptionServiceService {
    public robot1: RobotService;
    public robot2: RobotService;
    public gazebo: RobotService;
    server: any;
    constructor() {
        this.robot1 = new RobotService(process.env.ROBOT1_IP, RobotId.robot1);
        this.robot2 = new RobotService(process.env.ROBOT2_IP, RobotId.robot2);
        this.gazebo = new RobotService(process.env.GAZEBO_IP, RobotId.gazebo);
    }

    async subscribeToTopicRobot1(gateway: MissionCommandGateway){
        await this.robot1.subscribeToTopic(Topic.mission_status1, TopicType.mission_status, this.missionStatusCallback.bind(gateway));
        await this.robot1.subscribeToTopic(Topic.log_robot1, TopicType.log_message, this.logCallback.bind(gateway));
    }
    async subscribeToTopicRobot2(gateway: MissionCommandGateway){
        await this.robot2.subscribeToTopic(Topic.mission_status2, TopicType.mission_status, this.missionStatusCallback.bind(gateway));
        await this.robot2.subscribeToTopic(Topic.log_robot2, TopicType.log_message, this.logCallback.bind(gateway));
    }
    async subscribeToTopicGazebo(gateway: MissionCommandGateway){
        await this.gazebo.subscribeToTopic(Topic.gazebo_mission_status, TopicType.mission_status, this.missionStatusCallback.bind(gateway));
        await this.gazebo.subscribeToTopic(Topic.log_gazebo, TopicType.log_message, this.logCallback.bind(gateway));
    }

    async subscribeToTopicRobots(gateway: MissionCommandGateway) {
        await this.subscribeToTopicRobot1(gateway);
        await this.subscribeToTopicRobot2(gateway);
    }

    missionStatusCallback(message) {
        const robotStatus: RobotStatus = message.msg;
        this.server.emit('robotStatus', robotStatus);
    }

    logCallback(message) {
        const logMessage: LogMessage = message.msg;
        this.server.emit('log', logMessage);
    }

}
