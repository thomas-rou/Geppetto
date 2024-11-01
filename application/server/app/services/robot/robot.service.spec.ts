import { Test, TestingModule } from '@nestjs/testing';
import { RobotService } from './robot.service';
import { Logger } from '@nestjs/common';
import { RobotId } from '@common/enums/RobotId';
import { WebSocket } from 'ws';
import { Topic } from '@common/enums/Topic';
import { TopicType } from '@common/enums/TopicType';
import { RobotCommand } from '@common/enums/RobotCommand';

jest.mock('ws');

describe('RobotService', () => {
  let service: RobotService;
  let logger: Logger;
  let ws: WebSocket;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        RobotService,
        { provide: 'robotIp', useValue: '127.0.0.1' },
        { provide: 'robotNb', useValue: RobotId.robot1 },
        { provide: Logger, useValue: new Logger() },
      ],
    }).compile();

    service = module.get<RobotService>(RobotService);
    logger = module.get<Logger>(Logger);

    ws = new WebSocket('ws://127.0.0.1:9090');
    ws.readyState = WebSocket.OPEN;
    jest.spyOn(ws, 'send').mockImplementation((data) => {});
    jest.spyOn(ws, 'addEventListener').mockImplementation((event, listener) => {});

    Object.defineProperty(ws, 'onopen', {
      value: jest.fn(() => {
        ws.readyState = WebSocket.OPEN;
      }),
    });
    Object.defineProperty(ws, 'onerror', {
      value: jest.fn((error) => {}),
    });
    Object.defineProperty(ws, 'onclose', {
      value: jest.fn(() => {}),
    });

    service['ws'] = ws;

    jest.spyOn(console, 'log').mockImplementation(() => {});
    jest.spyOn(logger, 'error').mockImplementation(() => {});
  });

  it('should be defined', () => {
    expect(service).toBeDefined();
  });

  it('should have a logger', () => {
    expect(service['logger']).toBeDefined();
    expect(service['logger']).toBeInstanceOf(Logger);
  });

  it('should log messages', () => {
    const logSpy = jest.spyOn(service['logger'], 'log');
    service['logger'].log('Test message');
    expect(logSpy).toHaveBeenCalledWith('Test message');
  });

  it('should have robotIp defined', () => {
    expect(service['_robotIp']).toBeDefined();
    expect(service['_robotIp']).toBe('127.0.0.1');
  });

  it('should have robotNb defined', () => {
    expect(service['_robotNumber']).toBeDefined();
    expect(service['_robotNumber']).toBe(RobotId.robot1);
  });

  it('should subscribe to a topic', async () => {
    const subscribeSpy = jest.spyOn(service, 'subscribeToTopic');
    const handleIncomingMessage = jest.fn();
    await service.subscribeToTopic(Topic.start_mission, TopicType.start_mission, handleIncomingMessage);
    expect(subscribeSpy).toHaveBeenCalledWith(Topic.start_mission, TopicType.start_mission, handleIncomingMessage);
  });

  it('should publish to a topic', async () => {
    const publishSpy = jest.spyOn(service, 'publishToTopic');
    const message = {
      command: RobotCommand.StartMission,
      timestamp: new Date().toISOString(),
    };
    await service.publishToTopic(Topic.start_mission, TopicType.start_mission, message);
    expect(publishSpy).toHaveBeenCalledWith(Topic.start_mission, TopicType.start_mission, message);
  });

  it('should start a mission', async () => {
    const startMissionSpy = jest.spyOn(service, 'startMission');
    await service.startMission();
    expect(startMissionSpy).toHaveBeenCalled();
  });

  it('should stop a mission', async () => {
    const stopMissionSpy = jest.spyOn(service, 'stopMission');
    await service.stopMission();
    expect(stopMissionSpy).toHaveBeenCalled();
  });

  it('should identify the robot', async () => {
    const identifySpy = jest.spyOn(service, 'identify');
    await service.identify();
    expect(identifySpy).toHaveBeenCalled();
  });

  it('should handle incoming messages for subscribed topic', async () => {
    const handleIncomingMessage = jest.fn();
    await service.subscribeToTopic(Topic.start_mission, TopicType.start_mission, handleIncomingMessage);
    const messageEvent = {
      data: JSON.stringify({ topic: Topic.start_mission, message: 'test' }),
    };
    ws.addEventListener.mock.calls[0][1](messageEvent);
    expect(handleIncomingMessage).toHaveBeenCalledWith({ topic: Topic.start_mission, message: 'test' });
  });

  it('should identify robot based on robot number', async () => {
    const publishSpy = jest.spyOn(service, 'publishToTopic');
    service['_robotNumber'] = RobotId.robot2;
    await service.identify();
    expect(publishSpy).toHaveBeenCalledWith(Topic.identify_command2, TopicType.identify_robot, expect.any(Object));
  });
});
