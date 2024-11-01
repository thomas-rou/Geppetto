import { Test, TestingModule } from '@nestjs/testing';
import { MissionCommandGateway } from './mission-command.gateway';
import { SubscriptionServiceService } from '@app/services/subscription-service/subscription-service.service';
import { MissionService } from '@app/services/mission/mission.service';

describe('MissionCommandGateway', () => {
  let gateway: MissionCommandGateway;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        MissionCommandGateway,
        {
          provide: SubscriptionServiceService,
          useValue: {
            // Mock any methods used in the MissionCommandGateway
            robot1: {
              startMission: jest.fn(),
              stopMission: jest.fn(),
              identify: jest.fn(),
            },
            robot2: {
              startMission: jest.fn(),
              stopMission: jest.fn(),
              identify: jest.fn(),
            },
            gazebo: {
              startMission: jest.fn(),
              stopMission: jest.fn(),
              identify: jest.fn(),
            },
            subscribeToTopicRobots: jest.fn(),
            subscribeToTopicGazebo: jest.fn(),
          },
        },
        {
          provide: MissionService,
          useValue: {
            // Mock any methods used in the MissionCommandGateway
            createMission: jest.fn(),
            addLogToMission: jest.fn(),
            getAllMissions: jest.fn(),
          },
        },
      ],
    }).compile();

    gateway = module.get<MissionCommandGateway>(MissionCommandGateway);
  });

  it('should be defined', () => {
    expect(gateway).toBeDefined();
  });
});