import { Injectable } from '@nestjs/common';
import { MissionStatus } from '@common/interfaces/MissionStatus';
import { RobotService } from '../robot/robot.service';
import { Topic } from '@common/enums/Topic';
import { TopicType } from '@common/enums/TopicType';
@Injectable()
export class SubscriptionServiceService {
    // constructor(private robotService: RobotService) {}
    // missionStatus: MissionStatus;
    // public async subscribeToTopic() {
    //     // Subscription for robot 1
    //     await this.robotService.subscribeToTopic(Topic.mission_status1, TopicType.mission_status);

    //     // Subscription for robot 2
    //     await this.robotService.subscribeToTopic(Topic.mission_status2, TopicType.mission_status);

    //     // Subscription for gazebo
    // }
}
