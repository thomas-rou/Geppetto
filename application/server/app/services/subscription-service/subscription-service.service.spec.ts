import { Test, TestingModule } from '@nestjs/testing';
import { SubscriptionServiceService } from './subscription-service.service';
import { MissionService } from '../mission/mission.service';
import { RobotService } from '../robot/robot.service';
import { MissionCommandGateway } from '@app/gateways/mission-command/mission-command.gateway';
import { Topic } from '@common/enums/Topic';
import { TopicType } from '@common/enums/TopicType';
import { RobotId } from '@common/enums/RobotId';
import { RobotCommand } from '@common/enums/RobotCommand';

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
                        addMapToMission: jest.fn(),
                        updateTraveledDistance: jest.fn(),
                    },
                },
                {
                    provide: RobotService,
                    useValue: {
                        updateRobotController: jest.fn(),
                        robotNb: 2,
                        robotIp: '192.168.1.100',
                    },
                },
            ],
        }).compile();

        service = module.get<SubscriptionServiceService>(SubscriptionServiceService);
        missionService = module.get<MissionService>(MissionService);
        gateway = new MissionCommandGateway(service, missionService);

        jest.spyOn(service, 'updateRobotController').mockImplementation((robotData, filePath) => {
            return new Promise((resolve, reject) => {
                if (!robotData || !filePath) {
                    reject(new Error('Invalid data or path'));
                } else {
                    resolve();
                }
            });
        });

        service.robot1 = { subscribeToTopic: jest.fn() } as any;
        service.robot2 = { subscribeToTopic: jest.fn() } as any;
        service.gazebo = { subscribeToTopic: jest.fn() } as any;
    });

    it('should be defined', () => {
        expect(service).toBeDefined();
    });

    it('should subscribe to topics for robot1', async () => {
        await service.subscribeToTopicRobot1(gateway);
        expect(service.robot1.subscribeToTopic).toHaveBeenCalledWith(Topic.mission_status1, TopicType.mission_status, expect.any(Function));
        expect(service.robot1.subscribeToTopic).toHaveBeenCalledWith(Topic.log_robot1, TopicType.log_message, expect.any(Function));
        expect(service.robot1.subscribeToTopic).toHaveBeenCalledWith(Topic.physical_robot_map, TopicType.map, expect.any(Function));
        expect(service.robot1.subscribeToTopic).toHaveBeenCalledWith(
            Topic.robot1_pose_with_distance,
            TopicType.pose_with_distance,
            expect.any(Function),
        );
    });

    it('should subscribe to topics for robot2', async () => {
        await service.subscribeToTopicRobot2(gateway);
        expect(service.robot2.subscribeToTopic).toHaveBeenCalledWith(Topic.mission_status2, TopicType.mission_status, expect.any(Function));
        expect(service.robot2.subscribeToTopic).toHaveBeenCalledWith(Topic.log_robot2, TopicType.log_message, expect.any(Function));
        expect(service.robot2.subscribeToTopic).toHaveBeenCalledWith(
            Topic.robot2_pose_with_distance,
            TopicType.pose_with_distance,
            expect.any(Function),
        );
    });

    it('should subscribe to topics for gazebo', async () => {
        await service.subscribeToTopicGazebo(gateway);
        expect(service.gazebo.subscribeToTopic).toHaveBeenCalledWith(Topic.mission_status1, TopicType.mission_status, expect.any(Function));
        expect(service.gazebo.subscribeToTopic).toHaveBeenCalledWith(Topic.mission_status2, TopicType.mission_status, expect.any(Function));
        expect(service.gazebo.subscribeToTopic).toHaveBeenCalledWith(Topic.log_gazebo, TopicType.log_message, expect.any(Function));
        expect(service.gazebo.subscribeToTopic).toHaveBeenCalledWith(Topic.map, TopicType.map, expect.any(Function));
        expect(service.gazebo.subscribeToTopic).toHaveBeenCalledWith(
            Topic.gazebo1_pose_with_distance,
            TopicType.pose_with_distance,
            expect.any(Function),
        );
        expect(service.gazebo.subscribeToTopic).toHaveBeenCalledWith(
            Topic.gazebo2_pose_with_distance,
            TopicType.pose_with_distance,
            expect.any(Function),
        );
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
        service.missionStatusCallback(message);
        expect(service.server.emit).toHaveBeenCalledWith('robotStatus', message.msg);
    });

    it('should handle log callback', async () => {
        const message = { msg: { log: 'test log' } };
        service.server = { emit: jest.fn() };
        await service.logCallback(message);
        expect(service.server.emit).toHaveBeenCalledWith('log', message.msg);
        expect(missionService.addLogToMission).toHaveBeenCalledWith('test-mission-id', message.msg);
    });

    it('should handle map callback', () => {
        const message = { msg: { data: 'map data' } };
        service.server = { emit: jest.fn() };
        service.mapCallback(message);
        expect(service.server.emit).toHaveBeenCalledWith('liveMap', message.msg);
    });

    it('should mock updateRobotController and reject if invalid data or path is provided', async () => {
        await expect(service.updateRobotController(null, 'path/to/file')).rejects.toThrow('Invalid data or path');
        await expect(
            service.updateRobotController(
                { command: RobotCommand.UpdateControllerCode, code: 'new code', filename: 'yeyeyey', timestamp: 'rasboTIME' },
                '',
            ),
        ).rejects.toThrow('Invalid data or path');
    });
});
