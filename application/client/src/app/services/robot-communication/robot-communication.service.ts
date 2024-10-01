import { Injectable } from '@angular/core';
import { RobotCommand } from '@common/enums/SocketsEvents';
import { EndMission } from '@common/interfaces/EndMission';
import { StartMission } from '@common/interfaces/StartMission';
import { IdentifyRobot } from '@common/interfaces/IdentifyRobot';
import { UpdateRobot } from '@common/interfaces/UpdateRobot';
import { ReturnToBase } from '@common/interfaces/ReturnToBase';
import { UpdateControllerCode } from '@common/interfaces/UpdateControllerCode';
import { NotifyRobotsToCommunicate } from '@common/interfaces/NotifyRobotsToCommunicate';
import { FindFurthestRobot } from '@common/interfaces/FindFurthestRobot';
import { Observable, Subject } from 'rxjs';
import { io, Socket } from 'socket.io-client';
import { environment } from 'src/environments/environment';
import { RobotList } from '@common/enums/SocketsEvents';

@Injectable({
    providedIn: 'root',
})
export class RobotCommunicationService {
    private socket: Socket;

    private missionStatusSubject = new Subject<string>();
    private robotIdentificationSubject = new Subject<string>();
    private commandErrorSubject = new Subject<string>();
    private connectionStatusSubject = new Subject<boolean>();

    constructor() {
        this.socket = io(environment.serverUrlRoot, { transports: ['websocket'], upgrade: false });

        this.socket.on('missionStatus', (message: string) => {
            this.missionStatusSubject.next(message);
        });
        this.socket.on('robotIdentification', (message: string) => {
            this.robotIdentificationSubject.next(message);
        });
        this.socket.on('commandError', (message: string) => {
            this.commandErrorSubject.next(message);
        });

        this.socket.on('connect', () => {
            console.log('WebSocket connection established');
            this.connectionStatusSubject.next(true);
        });

        this.socket.on('disconnect', () => {
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

    startMission(orientation: number, position: { x: number; y: number }): void {
        this.startMissionRobot(orientation, position);
        this.startMissionGazebo(orientation, position);
    }

    startMissionRobot(orientation: number, position: { x: number; y: number }): void {
        const message: StartMission = {
            command: RobotCommand.StartMission,
            target: 'robot',
            mission_details: {
                orientation,
                position,
            },
            timestamp: new Date().toISOString(),
        };
        this.socket.emit(RobotCommand.StartMission, message);
    }

    endMission(): void {
        this.endMissionRobot();
        this.endMissionGazebo();
    }

    endMissionRobot(): void {
        const message: EndMission = {
            command: RobotCommand.EndMission,
            target: 'robot',
            timestamp: new Date().toISOString(),
        };
        this.socket.emit(RobotCommand.EndMission, message);
    }

    startMissionGazebo(orientation: number, position: { x: number; y: number }): void {
        const message: StartMission = {
            command: RobotCommand.StartMission,
            target: 'simulation',
            mission_details: {
                orientation,
                position,
            },
            timestamp: new Date().toISOString(),
        };
        this.socket.emit(RobotCommand.StartMission, message);
    }

    endMissionGazebo(): void {
        const message: EndMission = {
            command: RobotCommand.EndMission,
            target: 'simulation',
            timestamp: new Date().toISOString(),
        };
        this.socket.emit(RobotCommand.EndMission, message);
    }

    updateRobot(identifier: string, status: string, position: { x: number; y: number }): void {
        const message: UpdateRobot = {
            command: RobotCommand.UpdateRobot,
            identifier,
            status,
            position,
            timestamp: new Date().toISOString(),
        };
        this.socket.emit(RobotCommand.UpdateRobot, message);
    }

    returnToBase(): void {
        const message: ReturnToBase = {
            command: RobotCommand.ReturnToBase,
            timestamp: new Date().toISOString(),
        };
        this.socket.emit(RobotCommand.ReturnToBase, message);
    }

    updateControllerCode(newCode: string): void {
        const message: UpdateControllerCode = {
            command: RobotCommand.UpdateControllerCode,
            code: newCode,
            timestamp: new Date().toISOString(),
        };
        this.socket.emit(RobotCommand.UpdateControllerCode, message);
    }

    notifyRobotsToCommunicate(): void {
        const message: NotifyRobotsToCommunicate = {
            command: RobotCommand.InitiateP2P,
            timestamp: new Date().toISOString(),
        };
        this.socket.emit(RobotCommand.InitiateP2P, message);
    }

    findFurthestRobot(relativePoint: { x: number; y: number }): void {
        const message: FindFurthestRobot = {
            command: RobotCommand.FindFurthestRobot,
            relative_point: relativePoint,
            timestamp: new Date().toISOString(),
        };
        this.socket.emit(RobotCommand.FindFurthestRobot, message);
    }

    IdentifyRobot(target: RobotList): void {
        const message: IdentifyRobot = {
            command: 'identify_robot',
            target,
        };
        this.socket.emit(RobotCommand.IdentifyRobot, message);
    }

    onMessage(eventName: string): Observable<any> {
        return new Observable((observer) => {
            this.socket.on(eventName, (data: any) => {
                observer.next(data);
            });
        });
    }

    disconnect(): void {
        this.socket.disconnect();
    }
}
