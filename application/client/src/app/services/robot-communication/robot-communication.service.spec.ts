import { TestBed } from '@angular/core/testing';
import { RobotCommunicationService } from './robot-communication.service';
import { RobotManagementService } from '@app/services/robot-management/robot-management.service';
import { Socket } from 'socket.io-client';

describe('RobotCommunicationService', () => {
    let service: RobotCommunicationService;
    let robotManagementService: RobotManagementService;
    let socket: jasmine.SpyObj<Socket>;

    beforeEach(() => {
        socket = jasmine.createSpyObj('Socket', ['on', 'emit', 'disconnect', 'connect', 'close', 'send', 'open']);
        socket.id = 'mock-id';
        socket.connected = true;

        const robotManagementServiceMock = {
            robot1: { orientation: 90, position: { x: 1, y: 2 } },
            robot2: { orientation: 180, position: { x: 3, y: 4 } },
        };

        TestBed.configureTestingModule({
            providers: [RobotCommunicationService, { provide: RobotManagementService, useValue: robotManagementServiceMock }],
        });

        service = TestBed.inject(RobotCommunicationService);
        robotManagementService = TestBed.inject(RobotManagementService);
        service['socket'] = socket;
    });

    beforeEach(() => {
        socket.emit.calls.reset();
        socket.on.calls.reset();
        socket.disconnect.calls.reset();
    });

    it('should return robot1 and robot2 correctly from RobotManagementService', () => {
        expect(service.robot1).toEqual(robotManagementService.robot1);
        expect(service.robot2).toEqual(robotManagementService.robot2);
    });

    it('should emit startMission for robot and simulation', () => {
        service.startMission();
        expect(socket.emit).toHaveBeenCalledWith('start_mission', {
            command: 'start_mission',
            target: 'robot',
            mission_details: {
                orientation: robotManagementService.robot1.orientation,
                position: robotManagementService.robot1.position,
            },
            timestamp: jasmine.any(String),
        });
        expect(socket.emit).toHaveBeenCalledWith('start_mission', {
            command: 'start_mission',
            target: 'simulation',
            mission_details: {
                orientation: robotManagementService.robot1.orientation,
                position: robotManagementService.robot1.position,
            },
            timestamp: jasmine.any(String),
        });
    });

    it('should emit endMission for robot and simulation', () => {
        service.endMission();
        expect(socket.emit).toHaveBeenCalledWith('end_mission', {
            command: 'end_mission',
            target: 'robot',
            timestamp: jasmine.any(String),
        });
        expect(socket.emit).toHaveBeenCalledWith('end_mission', {
            command: 'end_mission',
            target: 'simulation',
            timestamp: jasmine.any(String),
        });
    });

    it('should emit updateRobot', () => {
        service.updateRobot('robot1', 'active', { x: 10, y: 20 });
        expect(socket.emit).toHaveBeenCalledWith('update', {
            command: 'update',
            identifier: 'robot1',
            status: 'active',
            position: { x: 10, y: 20 },
            timestamp: jasmine.any(String),
        });
    });

    it('should emit returnToBase', () => {
        service.returnToBase();
        expect(socket.emit).toHaveBeenCalledWith('return_to_base', {
            command: 'return_to_base',
            timestamp: jasmine.any(String),
        });
    });

    it('should emit updateControllerCode', () => {
        service.updateControllerCode('newCode');
        expect(socket.emit).toHaveBeenCalledWith('update_controller_code', {
            command: 'update_controller_code',
            code: 'newCode',
            timestamp: jasmine.any(String),
        });
    });

    it('should emit notifyRobotsToCommunicate', () => {
        service.notifyRobotsToCommunicate();
        expect(socket.emit).toHaveBeenCalledWith('initiate_p2p', {
            command: 'P2P',
            timestamp: jasmine.any(String),
        });
    });

    it('should emit findFurthestRobot', () => {
        service.findFurthestRobot({ x: 5, y: 5 });
        expect(socket.emit).toHaveBeenCalledWith('find_furthest', {
            command: 'find_furthest',
            relative_point: { x: 5, y: 5 },
            timestamp: jasmine.any(String),
        });
    });

    it('should emit identifyRobot', () => {
        service.identifyRobot('robot1');
        expect(socket.emit).toHaveBeenCalledWith('identify_robot', {
            command: 'identify_robot',
            target: 'robot1',
        });
    });

    it('should handle onMessage correctly', () => {
        const eventName = 'customEvent';
        const testMessage = { data: 'test' };

        socket.on.and.callFake((event: string, callback: (...args: any[]) => void): Socket => {
            if (event === eventName) {
                callback(testMessage);
            }
            return socket;
        });

        service.onMessage(eventName).subscribe((message) => {
            expect(message).toEqual(testMessage);
        });
    });

    it('should disconnect the socket', () => {
        service.disconnect();
        expect(socket.disconnect).toHaveBeenCalled();
    });

    it('should handle onMissionStatus observable', (done) => {
        service.onMissionStatus().subscribe((message) => {
            expect(message).toBe('mission-status');
            done();
        });
        service['missionStatusSubject'].next('mission-status');
    });

    it('should handle onRobotIdentification observable', (done) => {
        service.onRobotIdentification().subscribe((message) => {
            expect(message).toBe('robot-identification');
            done();
        });
        service['robotIdentificationSubject'].next('robot-identification');
    });

    it('should handle onCommandError observable', (done) => {
        service.onCommandError().subscribe((message) => {
            expect(message).toBe('command-error');
            done();
        });
        service['commandErrorSubject'].next('command-error');
    });

    it('should handle onConnectionStatus observable', (done) => {
        service.onConnectionStatus().subscribe((status) => {
            expect(status).toBe(true);
            done();
        });
        service['connectionStatusSubject'].next(true);
    });
});
