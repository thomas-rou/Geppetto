import { Injectable } from '@angular/core';
import {
    StartMission,
    EndMission,
    Update,
    ReturnToBase,
    UpdateControllerCode,
    P2PCommunication,
    IdentifyRobot,
    FindFurthestRobot
} from '@app/../../../SocketsEvents';
import { RobotCommandFromInterface } from '@app/../../../SocketsEvents';
import { Observable, Subject } from 'rxjs';
import { io, Socket } from 'socket.io-client';
import { environment } from 'src/environments/environment';



@Injectable({
    providedIn: 'root',
})
export class RobotCommunicationService {
    private socket: Socket;

    private missionStatusSubject = new Subject<string>();
    private robotIdentificationSubject = new Subject<string>();
    private commandErrorSubject = new Subject<string>();

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

    startMission(orientation: number, position: { x: number; y: number }): void {
        const message = new StartMission("robots", orientation, position.x, position.y, new Date().toISOString());
        this.socket.emit(RobotCommandFromInterface.StartMission, message);
    }

    endMission(): void {
        const message = new EndMission("robots", new Date().toISOString());
        this.socket.emit(RobotCommandFromInterface.EndMission, message);
    }

    startMissionGazebo(orientation: number, position: { x: number; y: number }): void {
        const message = new StartMission("sim", orientation, position.x, position.y, new Date().toISOString());
        this.socket.emit(RobotCommandFromInterface.StartMission, message);
    }

    endMissionGazebo(): void {
        const message = new EndMission("sim", new Date().toISOString());
        this.socket.emit(RobotCommandFromInterface.EndMission, message);
    }

    updateRobot(identifier: string, status: string, position: { x: number; y: number }): void {
        const message = new Update(identifier, status, position.x, position.y, new Date().toISOString());
        this.socket.emit(RobotCommandFromInterface.UpdateRobot, message);
    }

    returnToBase(): void {
        const message = new ReturnToBase(new Date().toISOString());
        this.socket.emit(RobotCommandFromInterface.ReturnToBase, message);
    }

    updateControllerCode(newCode: string): void {
        const message = new UpdateControllerCode(newCode, new Date().toISOString());
        this.socket.emit(RobotCommandFromInterface.UpdateControllerCode, message);
    }

    notifyRobotsToCommunicate(): void {
        const message = new P2PCommunication(new Date().toISOString());
        this.socket.emit(RobotCommandFromInterface.NotifyRobotsToCommunicate, message);
    }

    findFurthestRobot(relativePoint: { x: number; y: number }): void {
        const message = new FindFurthestRobot(relativePoint.x, relativePoint.y, new Date().toISOString());
        this.socket.emit(RobotCommandFromInterface.FindFurthestRobot, message);
    }

    identifyRobot(target: "1" | "2"): void {
        const message = new IdentifyRobot(target);
        this.socket.emit(RobotCommandFromInterface.IdentifyRobot, message);
    }

    onMessage(eventName: string): Observable<any> {
        return new Observable(observer => {
            this.socket.on(eventName, (data: any) => {
                observer.next(data);
            });
        });
    }

    disconnect(): void {
        this.socket.disconnect();
    }
}
