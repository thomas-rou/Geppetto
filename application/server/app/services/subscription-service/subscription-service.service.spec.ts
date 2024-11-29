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
    const spyMap = jest.spyOn(service.robot1, 'subscribeToTopic');
    const spyPose = jest.spyOn(service.robot1, 'subscribeToTopic');

    await service.subscribeToTopicRobot1(gateway);

    expect(spyMissionStatus).toHaveBeenCalledWith(Topic.mission_status1, TopicType.mission_status, expect.any(Function));
    expect(spyLog).toHaveBeenCalledWith(Topic.log_robot1, TopicType.log_message, expect.any(Function));
    expect(spyMap).toHaveBeenCalledWith(Topic.physical_robot_map, TopicType.map, expect.any(Function));
    expect(spyPose).toHaveBeenCalledWith(Topic.robot1_pose, TopicType.pose, expect.any(Function));
  });

  it('should subscribe to topics for robot2', async () => {
    const spyMissionStatus = jest.spyOn(service.robot2, 'subscribeToTopic');
    const spyLog = jest.spyOn(service.robot2, 'subscribeToTopic');
    const spyPose = jest.spyOn(service.robot2, 'subscribeToTopic');


    await service.subscribeToTopicRobot2(gateway);

    expect(spyMissionStatus).toHaveBeenCalledWith(Topic.mission_status2, TopicType.mission_status, expect.any(Function));
    expect(spyLog).toHaveBeenCalledWith(Topic.log_robot2, TopicType.log_message, expect.any(Function));
    expect(spyPose).toHaveBeenCalledWith(Topic.robot2_pose, TopicType.pose, expect.any(Function));
  });

  it('should subscribe to topics for gazebo', async () => {
    const spyMissionStatus1 = jest.spyOn(service.gazebo, 'subscribeToTopic');
    const spyMissionStatus2 = jest.spyOn(service.gazebo, 'subscribeToTopic');
    const spyLog = jest.spyOn(service.gazebo, 'subscribeToTopic');
    const spyMap = jest.spyOn(service.gazebo, 'subscribeToTopic');
    const spyPose1 = jest.spyOn(service.gazebo, 'subscribeToTopic');
    const spyPose2 = jest.spyOn(service.gazebo, 'subscribeToTopic');

    await service.subscribeToTopicGazebo(gateway);

    expect(spyMissionStatus1).toHaveBeenCalledWith(Topic.mission_status1, TopicType.mission_status, expect.any(Function));
    expect(spyMissionStatus2).toHaveBeenCalledWith(Topic.mission_status2, TopicType.mission_status, expect.any(Function));
    expect(spyLog).toHaveBeenCalledWith(Topic.log_gazebo, TopicType.log_message, expect.any(Function));
    expect(spyMap).toHaveBeenCalledWith(Topic.map, TopicType.map, expect.any(Function));
    expect(spyPose1).toHaveBeenCalledWith(Topic.gazebo1_pose, TopicType.pose, expect.any(Function));
    expect(spyPose2).toHaveBeenCalledWith(Topic.gazebo2_pose, TopicType.pose, expect.any(Function));
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

  it('should handle map callback', () => {
    const message = { msg: { data: 'map data' } };
    service.server = { emit: jest.fn() };
    const spyEmit = jest.spyOn(service.server, 'emit');

    service.mapCallback(message);

    expect(spyEmit).toHaveBeenCalledWith('liveMap', message.msg);
  });

  it('should handle robot pose callback', () => {
    const message = { msg: { x: 1, y: 2, z: 3 }, topic: 'robotPose' };
    service.server = { emit: jest.fn() };
    const spyEmit = jest.spyOn(service.server, 'emit');

    service.robotPoseCallback(message);

    expect(spyEmit).toHaveBeenCalledWith('robotPose', { ...message.msg, topic: message.topic });
  });
});