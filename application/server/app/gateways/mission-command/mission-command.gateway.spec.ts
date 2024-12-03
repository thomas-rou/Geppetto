import { Test, TestingModule } from '@nestjs/testing';
import { MissionCommandGateway } from './mission-command.gateway';
import { SubscriptionServiceService } from '@app/services/subscription-service/subscription-service.service';
import { MissionService } from '@app/services/mission/mission.service';
import { LogService } from '@app/services/log/log.service';
import { Server, Socket } from 'socket.io';
import { RobotId } from '@common/enums/RobotId';
import { LogType } from '@common/enums/LogType';
import { RobotCommand } from '@common/enums/RobotCommand';
import { ClientCommand } from '@common/enums/ClientCommand';
import { StartMission } from '@common/interfaces/StartMission';
import { MissionDetails } from '@common/interfaces/MissionDetails';
import { MissionType } from '@common/enums/MissionType';
import { EndMission } from '@common/interfaces/EndMission';
import { IdentifyRobot } from '@common/interfaces/IdentifyRobot';
import { Mission } from '@app/model/database/Mission';

describe('MissionCommandGateway', () => {
    let gateway: MissionCommandGateway;
    let subscriptionService: SubscriptionServiceService;
    let missionService: MissionService;
    let logger: LogService;
    let server: { emit: jest.Mock };

    beforeEach(async () => {
        server = { emit: jest.fn() };

        const module: TestingModule = await Test.createTestingModule({
            imports: [MissionService],
            providers: [
                MissionCommandGateway,
                {
                    provide: SubscriptionServiceService,
                    useValue: {
                        robot1: { startMission: jest.fn(), stopMission: jest.fn(), identify: jest.fn() },
                        robot2: { startMission: jest.fn(), stopMission: jest.fn(), identify: jest.fn() },
                        gazebo: { startMission: jest.fn(), stopMission: jest.fn() },
                        subscribeToTopicRobots: jest.fn(),
                        subscribeToTopicGazebo: jest.fn(),
                    },
                },
                {
                    provide: LogService,
                    useValue: {
                        logToClient: jest.fn(),
                    },
                },
            ],
        }).compile();

        gateway = module.get<MissionCommandGateway>(MissionCommandGateway);
        subscriptionService = module.get<SubscriptionServiceService>(SubscriptionServiceService);
        missionService = module.get<MissionService>(MissionService);
        logger = module.get<LogService>(LogService);

        gateway.server = server as any;
    });

    it('should be defined', () => {
        expect(gateway).toBeDefined();
    });

    it('should start mission for robots', async () => {
        const client = { id: 'client1', emit: jest.fn() } as unknown as Socket;
        const payload: StartMission = {
            target: [RobotId.robot1, RobotId.robot2],
            command: RobotCommand.StartMission,
            mission_details: {
                orientation1: 1,
                position1: { x: 1, y: 1 },
                orientation2: 1,
                position2: { x: 1, y: 1 },
            },
            timestamp: new Date().toISOString(),
        };
        await gateway.startMissionRobots(client, payload);
        expect(subscriptionService.robot1.startMission).toHaveBeenCalled();
        expect(subscriptionService.robot2.startMission).toHaveBeenCalled();
    });

    it('should stop mission for robots', async () => {
        const client = { id: 'client1', emit: jest.fn() } as unknown as Socket;
        const payload: EndMission = {
            target: [RobotId.robot1, RobotId.robot2],
            command: RobotCommand.EndMission,
            timestamp: new Date().toISOString(),
        };
        await gateway.stopMissionFromRobots(client, payload);
        expect(subscriptionService.robot1.stopMission).toHaveBeenCalled();
        expect(subscriptionService.robot2.stopMission).toHaveBeenCalled();
    });

    it('should identify robot', async () => {
        const client = { id: 'client1', emit: jest.fn() } as unknown as Socket;
        const payload: IdentifyRobot = {
            target: RobotId.robot2,
            command: RobotCommand.IdentifyRobot,
        };
        await gateway.identifyRobot(client, payload);
        expect(subscriptionService.robot2.identify).toHaveBeenCalled();
        expect(server.emit).toHaveBeenCalledWith('robotIdentification', 'Robot 2 was identified');
    });

    it('should get mission logs', async () => {
        const client = { id: 'client1', emit: jest.fn() } as unknown as Socket;
        const missions: Mission[] = [
            {
                id: '1',
                logs: [
                    { source: 'system', log_type: 'info', date: new Date().toISOString(), message: 'Log entry 1' },
                    { source: 'system', log_type: 'info', date: new Date().toISOString(), message: 'Log entry 2' },
                ],
                map: [],
                missionType: MissionType.GAZEBO_SIMULATION,
                missionDuration: '2h 30m',
                traveledDistance: 1500,
                robots: ['robot1', 'robot2'],
            },
            {
                id: '2',
                logs: [{ source: 'system', log_type: 'info', date: new Date().toISOString(), message: 'Log entry 1' }],
                map: [],
                missionType: MissionType.GAZEBO_SIMULATION,
                missionDuration: '1h 0m',
                traveledDistance: 800,
                robots: ['robot1'],
            },
        ];
        jest.spyOn(missionService, 'getAllMissions').mockResolvedValue(missions);
        await gateway.getMissionLogs(client);
        expect(client.emit).toHaveBeenCalledWith(ClientCommand.MissionLogs, missions);
    });
});
