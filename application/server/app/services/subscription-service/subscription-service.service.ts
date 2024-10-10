import { Injectable } from '@nestjs/common';
import { MissionStatus } from '@common/interfaces/MissionStatus';
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
    }
    async subscribeToTopicRobot2(gateway: MissionCommandGateway){
        await this.robot2.subscribeToTopic(Topic.mission_status2, TopicType.mission_status, this.missionStatusCallback.bind(gateway));
    }
    async subscribeToTopicGazebo(gateway: MissionCommandGateway){
    }

    async subscribeToTopic(gateway: MissionCommandGateway) {
        await this.subscribeToTopicRobot1(gateway);
        await this.subscribeToTopicRobot2(gateway);
        await this.subscribeToTopicGazebo(gateway);
    }

    missionStatusCallback(message) {
        const missionStatus: MissionStatus = message.msg;
        this.server.emit('missionStatus', missionStatus);
    }
}
