import { Injectable } from '@angular/core';
import { RobotManagementService } from '@app/services/robot-management/robot-management.service';
import { RobotCommandFromInterface } from '@common/enums/SocketsEvents';
import { EndMission } from '@common/interfaces/EndMission';
import { StartMission } from '@common/interfaces/StartMission';
import { IdentifyRobot } from '@common/interfaces/IdentifyRobot';
import { UpdateRobot } from '@common/interfaces/UpdateRobot';
import { ReturnToBase } from '@common/interfaces/ReturnToBase';
import { UpdateControllerCode } from '@common/interfaces/UpdateControllerCode';
import { NotifyRobotsToCommunicate } from '@common/interfaces/NotifyRobotsToCommunicate';
import { FindFurthestRobot } from '@common/interfaces/FindFurthestRobot';
import { Observable, Subject } from 'rxjs';
import { SocketHandlerService } from '@app/services/socket-handler/socket-handler.service';

@Injectable({
    providedIn: 'root',
})
export class RobotCommunicationService {
    private missionStatusSubject = new Subject<string>();
    private robotIdentificationSubject = new Subject<string>();
    private commandErrorSubject = new Subject<string>();
    private connectionStatusSubject = new Subject<boolean>();

    constructor(
        public socketService: SocketHandlerService,
        private robotManagementService: RobotManagementService,
    ) {
        this.connect();
    }

    get robot1() {
        return this.robotManagementService.robot1;
    }
    get robot2() {
        return this.robotManagementService.robot2;
    }

    connect() {
        if (!this.socketService.isSocketAlive()) {
            this.socketService.connect();
            this.handleConnect();
            this.handleMissionStatus();
            this.handleRobotIdentification();
            this.handleCommandError();
        }
    }

    handleConnect() {
        this.socketService.on('connect', () => {
            console.log('WebSocket connection established');
            this.connectionStatusSubject.next(true);
        });
    }

    handleMissionStatus() {
        this.socketService.on('missionStatus', (message: string) => {
            this.missionStatusSubject.next(message);
        });
    }

    handleRobotIdentification() {
        this.socketService.on('robotIdentification', (message: string) => {
            this.robotIdentificationSubject.next(message);
        });
    }

    handleCommandError() {
        this.socketService.on('commandError', (message: string) => {
            this.commandErrorSubject.next(message);
        });
    }

    handleDisconnect() {
        this.socketService.on('disconnect', () => {
            console.log('WebSocket connection lost');
            this.connectionStatusSubject.next(false);
        });
    }

    onMissionStatus(): Observable<string> {
        return this.missionStatusSubject.asObservable();
    }

    onRobotIdentification(): Observable<string> {
        return this.robotIdentificationSubject.asObservable();
    }

    onCommandError(): Observable<string> {
        return this.commandErrorSubject.asObservable();
    }

    onConnectionStatus(): Observable<boolean> {
        return this.connectionStatusSubject.asObservable();
    }

    startMission(): void {
        this.startMissionRobot();
        this.startMissionGazebo();
    }

    startMissionRobot(): void {
        const message: StartMission = {
            command: 'start_mission',
            target: 'robot',
            mission_details: {
                orientation: this.robot1.orientation,
                position: this.robot1.position,
            },
            timestamp: new Date().toISOString(),
        };
        this.socketService.send(RobotCommandFromInterface.StartMission, message);
    }

    endMission(): void {
        this.endMissionRobot();
        this.endMissionGazebo();
    }

    endMissionRobot(): void {
        const message: EndMission = {
            command: 'end_mission',
            target: 'robot',
            timestamp: new Date().toISOString(),
        };
        this.socketService.send(RobotCommandFromInterface.EndMission, message);
    }

    startMissionGazebo(): void {
        const message: StartMission = {
            command: 'start_mission',
            target: 'simulation',
            mission_details: {
                orientation: this.robot1.orientation,
                position: this.robot1.position,
            },
            timestamp: new Date().toISOString(),
        };
        this.socketService.send(RobotCommandFromInterface.StartMission, message);
    }

    endMissionGazebo(): void {
        const message: EndMission = {
            command: 'end_mission',
            target: 'simulation',
            timestamp: new Date().toISOString(),
        };
        this.socketService.send(RobotCommandFromInterface.EndMission, message);
    }

    updateRobot(identifier: string, status: string, position: { x: number; y: number }): void {
        const message: UpdateRobot = {
            command: 'update',
            identifier,
            status,
            position,
            timestamp: new Date().toISOString(),
        };
        this.socketService.send(RobotCommandFromInterface.UpdateRobot, message);
    }

    returnToBase(): void {
        const message: ReturnToBase = {
            command: 'return_to_base',
            timestamp: new Date().toISOString(),
        };
        this.socketService.send(RobotCommandFromInterface.ReturnToBase, message);
    }

    updateControllerCode(newCode: string): void {
        const message: UpdateControllerCode = {
            command: 'update_controller_code',
            code: newCode,
            timestamp: new Date().toISOString(),
        };
        this.socketService.send(RobotCommandFromInterface.UpdateControllerCode, message);
    }

    notifyRobotsToCommunicate(): void {
        const message: NotifyRobotsToCommunicate = {
            command: 'P2P',
            timestamp: new Date().toISOString(),
        };
        this.socketService.send(RobotCommandFromInterface.NotifyRobotsToCommunicate, message);
    }

    findFurthestRobot(relativePoint: { x: number; y: number }): void {
        const message: FindFurthestRobot = {
            command: 'find_furthest',
            relative_point: relativePoint,
            timestamp: new Date().toISOString(),
        };
        this.socketService.send(RobotCommandFromInterface.FindFurthestRobot, message);
    }

    identifyRobot(target: string): void {
        const message: IdentifyRobot = {
            command: 'identify_robot',
            target,
        };
        this.socketService.send(RobotCommandFromInterface.IdentifyRobot, message);
    }

    onMessage(eventName: string): Observable<unknown> {
        return new Observable((observer) => {
            this.socketService.on(eventName, (data: any) => {
                observer.next(data);
            });
        });
    }

    disconnect(): void {
        this.socketService.disconnect();
    }
}
