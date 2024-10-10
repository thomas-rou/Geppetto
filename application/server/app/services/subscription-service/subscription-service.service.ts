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
    constructor() {
        this.robot1 = new RobotService(process.env.ROBOT1_IP, RobotId.robot1);
        this.robot2 = new RobotService(process.env.ROBOT2_IP, RobotId.robot2);
        this.gazebo = new RobotService(process.env.GAZEBO_IP, RobotId.gazebo);
    }
    missionStatus: MissionStatus;
    public async subscribeToTopic(gateway: MissionCommandGateway) {
        // Subscription for robot 1
        await this.robot1.subscribeToTopic(Topic.mission_status1, TopicType.mission_status, this.missionStatusCallback.bind(gateway));

        // Subscription for robot 2
        await this.robot2.subscribeToTopic(Topic.mission_status2, TopicType.mission_status, this.missionStatusCallback.bind(gateway));

        // Subscription for gazebo
    }

    missionStatusCallback(message) {
        this.missionStatus = message;
    }
}
