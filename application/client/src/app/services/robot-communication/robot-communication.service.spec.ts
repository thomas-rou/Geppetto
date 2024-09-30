import { TestBed } from '@angular/core/testing';
import { RobotCommunicationService } from './robot-communication.service';
import { RobotManagementService } from '@app/services/robot-management/robot-management.service';
import { SocketTestHelper } from '@app/classes/socket-test-helper/socket-test-helper';
import { Socket } from 'socket.io-client';
import { SocketHandlerService } from '@app/services/socket-handler/socket-handler.service';
class SocketHandlerServiceMock extends SocketHandlerService {
    // Override connect() is required to not actually connect the socket
    // eslint-disable-next-line  @typescript-eslint/no-empty-function
    override connect() {}
}

describe('RobotCommunicationService', () => {
    let service: RobotCommunicationService;
    let robotManagementService: RobotManagementService;
    let socketSpy: SocketHandlerServiceMock;
    let socketHelper: SocketTestHelper;
    let onSpy: jasmine.Spy<() => void>;
    let sendSpy: unknown;

    const testMessage = { data: 'test' };

    beforeEach(() => {
        socketHelper = new SocketTestHelper();
        socketSpy = new SocketHandlerServiceMock();
        socketSpy.socket = socketHelper as unknown as Socket;

        onSpy = spyOn(socketSpy, 'on').and.callThrough();
        sendSpy = spyOn(socketSpy, 'send').and.callThrough();

        const robotManagementServiceMock = {
            robot1: { orientation: 90, position: { x: 1, y: 2 } },
            robot2: { orientation: 180, position: { x: 3, y: 4 } },
        };

        TestBed.configureTestingModule({
            providers: [
                { provide: SocketHandlerService, useValue: socketSpy },
                { provide: RobotManagementService, useValue: robotManagementServiceMock },
            ],
        });

        service = TestBed.inject(RobotCommunicationService);
        robotManagementService = TestBed.inject(RobotManagementService);
    });

    it('should return robot1 and robot2 correctly from RobotManagementService', () => {
        expect(service.robot1).toEqual(robotManagementService.robot1);
        expect(service.robot2).toEqual(robotManagementService.robot2);
    });

    it('should emit startMission for robot and simulation', () => {
        service.startMission();
        expect(sendSpy).toHaveBeenCalledWith('start_mission', {
            command: 'start_mission',
            target: 'robot',
            mission_details: {
                orientation: robotManagementService.robot1.orientation,
                position: robotManagementService.robot1.position,
            },
            timestamp: jasmine.any(String),
        });

        expect(sendSpy).toHaveBeenCalledWith('start_mission', {
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
        expect(sendSpy).toHaveBeenCalledWith('end_mission', {
            command: 'end_mission',
            target: 'robot',
            timestamp: jasmine.any(String),
        });
        expect(sendSpy).toHaveBeenCalledWith('end_mission', {
            command: 'end_mission',
            target: 'simulation',
            timestamp: jasmine.any(String),
        });
    });

    it('should emit updateRobot', () => {
        service.updateRobot('robot1', 'active', { x: 10, y: 20 });
        expect(sendSpy).toHaveBeenCalledWith('update', {
            command: 'update',
            identifier: 'robot1',
            status: 'active',
            position: { x: 10, y: 20 },
            timestamp: jasmine.any(String),
        });
    });

    it('should emit returnToBase', () => {
        service.returnToBase();
        expect(sendSpy).toHaveBeenCalledWith('return_to_base', {
            command: 'return_to_base',
            timestamp: jasmine.any(String),
        });
    });

    it('should emit updateControllerCode', () => {
        service.updateControllerCode('newCode');
        expect(sendSpy).toHaveBeenCalledWith('update_controller_code', {
            command: 'update_controller_code',
            code: 'newCode',
            timestamp: jasmine.any(String),
        });
    });

    it('should emit notifyRobotsToCommunicate', () => {
        service.notifyRobotsToCommunicate();
        expect(sendSpy).toHaveBeenCalledWith('initiate_p2p', {
            command: 'P2P',
            timestamp: jasmine.any(String),
        });
    });

    it('should emit findFurthestRobot', () => {
        service.findFurthestRobot({ x: 5, y: 5 });
        expect(sendSpy).toHaveBeenCalledWith('find_furthest', {
            command: 'find_furthest',
            relative_point: { x: 5, y: 5 },
            timestamp: jasmine.any(String),
        });
    });

    it('should emit identifyRobot', () => {
        service.identifyRobot('robot1');
        expect(sendSpy).toHaveBeenCalledWith('identify_robot', {
            command: 'identify_robot',
            target: 'robot1',
        });
    });

    it('should handle onMessage correctly', () => {
        const eventName = 'customEvent';
        service.onMessage(eventName).subscribe((message) => {
            expect(message).toBeFalsy();
        });

        socketHelper.peerSideEmit(eventName);

        service.onMessage(eventName).subscribe((message) => {
            expect(message).toEqual(testMessage);
        });
    });

    it('should disconnect the socket', () => {
        const disconnectSpy = spyOn(socketSpy, 'disconnect').and.callThrough();
        service.disconnect();
        expect(disconnectSpy).toHaveBeenCalled();
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

    it('handleMissionStatus() should receive connection status', () => {
        service.handleConnect();
        socketHelper.peerSideEmit('connect');

        expect(onSpy).toHaveBeenCalled();

        service['connectionStatusSubject'].subscribe((value: boolean) => {
            expect(value).toBe(true);
        });
    });

    it('handleMissionStatus() should receive new mission status', () => {
        const message = 'mission status';

        service.handleMissionStatus();
        socketHelper.peerSideEmit('missionStatus', message);

        expect(onSpy).toHaveBeenCalled();

        service['missionStatusSubject'].subscribe((value: string) => {
            expect(value).toBe(message);
        });
    });

    it('handleRobotIdentification() should receive identification status', () => {
        const message = 'identification status';

        service.handleRobotIdentification();
        socketHelper.peerSideEmit('robotIdentification', message);

        expect(onSpy).toHaveBeenCalled();

        service['robotIdentificationSubject'].subscribe((value: string) => {
            expect(value).toBe(message);
        });
    });

    it('handleCommandError() should receive error status', () => {
        const message = 'error status';

        service.handleCommandError();
        socketHelper.peerSideEmit('commandError', message);

        expect(onSpy).toHaveBeenCalled();

        service['commandErrorSubject'].subscribe((value: string) => {
            expect(value).toBe(message);
        });
    });

    it('handleDisconnect() should receive disconnection status', () => {
        service.handleDisconnect();
        socketHelper.peerSideEmit('disconnect');

        expect(onSpy).toHaveBeenCalled();

        service['connectionStatusSubject'].subscribe((value: boolean) => {
            expect(value).toBe(false);
        });
    });
});
