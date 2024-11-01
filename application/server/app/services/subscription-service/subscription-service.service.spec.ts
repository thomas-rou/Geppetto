import { Test, TestingModule } from '@nestjs/testing';
import { SubscriptionServiceService } from './subscription-service.service';
import { MissionService } from '../mission/mission.service';
import { RobotService } from '../robot/robot.service';
import { MissionCommandGateway } from '@app/gateways/mission-command/mission-command.gateway';
import { Topic } from '@common/enums/Topic';
import { TopicType } from '@common/enums/TopicType';
import { RobotId } from '@common/enums/RobotId';

describe('SubscriptionServiceService', () => {
  let service: SubscriptionServiceService;
  let missionService: MissionService;
  let gateway: MissionCommandGateway;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        SubscriptionServiceService,
        {
          provide: MissionService,
          useValue: {
            addLogToMission: jest.fn(),
            missionId: 'test-mission-id',
          },
        },
        {
          provide: RobotService,
          useValue: {
            subscribeToTopic: jest.fn(),
          },
        },
      ],
    }).compile();

    service = module.get<SubscriptionServiceService>(SubscriptionServiceService);
    missionService = module.get<MissionService>(MissionService);
    gateway = new MissionCommandGateway(service, missionService);

    service.robot1 = {
      subscribeToTopic: jest.fn(),
    } as any;
    service.robot2 = {
      subscribeToTopic: jest.fn(),
    } as any;
    service.gazebo = {
      subscribeToTopic: jest.fn(),
    } as any;
  });

  it('should be defined', () => {
    expect(service).toBeDefined();
  });

  it('should subscribe to topics for robot1', async () => {
    const spyMissionStatus = jest.spyOn(service.robot1, 'subscribeToTopic');
    const spyLog = jest.spyOn(service.robot1, 'subscribeToTopic');

    await service.subscribeToTopicRobot1(gateway);

    expect(spyMissionStatus).toHaveBeenCalledWith(Topic.mission_status1, TopicType.mission_status, expect.any(Function));
    expect(spyLog).toHaveBeenCalledWith(Topic.log_robot1, TopicType.log_message, expect.any(Function));
  });

  it('should subscribe to topics for robot2', async () => {
    const spyMissionStatus = jest.spyOn(service.robot2, 'subscribeToTopic');
    const spyLog = jest.spyOn(service.robot2, 'subscribeToTopic');

    await service.subscribeToTopicRobot2(gateway);

    expect(spyMissionStatus).toHaveBeenCalledWith(Topic.mission_status2, TopicType.mission_status, expect.any(Function));
    expect(spyLog).toHaveBeenCalledWith(Topic.log_robot2, TopicType.log_message, expect.any(Function));
  });

  it('should subscribe to topics for gazebo', async () => {
    const spyMissionStatus1 = jest.spyOn(service.gazebo, 'subscribeToTopic');
    const spyMissionStatus2 = jest.spyOn(service.gazebo, 'subscribeToTopic');
    const spyLog = jest.spyOn(service.gazebo, 'subscribeToTopic');

    await service.subscribeToTopicGazebo(gateway);

    expect(spyMissionStatus1).toHaveBeenCalledWith(Topic.mission_status1, TopicType.mission_status, expect.any(Function));
    expect(spyMissionStatus2).toHaveBeenCalledWith(Topic.mission_status2, TopicType.mission_status, expect.any(Function));
    expect(spyLog).toHaveBeenCalledWith(Topic.log_gazebo, TopicType.log_message, expect.any(Function));
  });

  it('should subscribe to topics for all robots', async () => {
    const spyRobot1 = jest.spyOn(service, 'subscribeToTopicRobot1');
    const spyRobot2 = jest.spyOn(service, 'subscribeToTopicRobot2');

    await service.subscribeToTopicRobots(gateway);

    expect(spyRobot1).toHaveBeenCalledWith(gateway);
    expect(spyRobot2).toHaveBeenCalledWith(gateway);
  });

  it('should handle mission status callback', () => {
    const message = { msg: { status: 'active' } };
    service.server = { emit: jest.fn() };
    const spyEmit = jest.spyOn(service.server, 'emit');

    service.missionStatusCallback(message);

    expect(spyEmit).toHaveBeenCalledWith('robotStatus', message.msg);
  });

  it('should handle log callback', async () => {
    const message = { msg: { log: 'test log' } };
    service.server = { emit: jest.fn() };
    const spyEmit = jest.spyOn(service.server, 'emit');
    const spyAddLog = jest.spyOn(missionService, 'addLogToMission');

    await service.logCallback(message);

    expect(spyEmit).toHaveBeenCalledWith('log', message.msg);
    expect(spyAddLog).toHaveBeenCalledWith('test-mission-id', message.msg);
  });
});
