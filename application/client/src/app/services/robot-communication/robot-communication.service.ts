import { Injectable } from '@angular/core';
import { RobotCommandFromInterface } from '@app/../../../common/enums/SocketsEvents';
import { EndMission } from '@app/../../../common/interfaces/EndMission';
import { StartMission } from '@app/../../../common/interfaces/StartMission';
import { IdentifyRobot } from '@app/../../../common/interfaces/IdentifyRobot';
import { UpdateRobot } from '@app/../../../common/interfaces/UpdateRobot';
import { ReturnToBase } from '@app/../../../common/interfaces/ReturnToBase';
import { UpdateControllerCode } from '@app/../../../common/interfaces/UpdateControllerCode';
import { NotifyRobotsToCommunicate } from '@app/../../../common/interfaces/NotifyRobotsToCommunicate';
import { FindFurthestRobot } from '@app/../../../common/interfaces/FindFurthestRobot';
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
        this.startMissionRobot(orientation, position);
        this.startMissionGazebo(orientation, position);
    }

    startMissionRobot(orientation: number, position: { x: number; y: number }): void {
        const message: StartMission = {
            command: "start_mission",
            target: "robot",
            mission_details: {
                orientation,
                position
            },
            timestamp: new Date().toISOString()
        };
        this.socket.emit(RobotCommandFromInterface.StartMission, message);
    }

    endMission(): void {
        this.endMissionRobot();
        this.endMissionGazebo();
    }

    endMissionRobot(): void {
        const message: EndMission = {
            command: "end_mission",
            target: "robot",
            timestamp: new Date().toISOString()
        };
        this.socket.emit(RobotCommandFromInterface.EndMission, message);
    }

    startMissionGazebo(orientation: number, position: { x: number; y: number }): void {
        const message: StartMission = {
            command: "start_mission",
            target: "sim",
            mission_details: {
                orientation,
                position
            },
            timestamp: new Date().toISOString()
        };
        this.socket.emit(RobotCommandFromInterface.StartMission, message);
    }

    endMissionGazebo(): void {
        const message: EndMission = {
            command: "end_mission",
            target: "sim",
            timestamp: new Date().toISOString()
        };
        this.socket.emit(RobotCommandFromInterface.EndMission, message);
    }

    updateRobot(identifier: string, status: string, position: { x: number; y: number }): void {
        const message: UpdateRobot = {
            command: "update",
            identifier,
            status,
            position,
            timestamp: new Date().toISOString()
        };
        this.socket.emit(RobotCommandFromInterface.UpdateRobot, message);
    }

    returnToBase(): void {
        const message: ReturnToBase = {
            command: "return_to_base",
            timestamp: new Date().toISOString()
        };
        this.socket.emit(RobotCommandFromInterface.ReturnToBase, message);
    }

    updateControllerCode(newCode: string): void {
        const message: UpdateControllerCode = {
            command: "update_controller_code",
            code: newCode,
            timestamp: new Date().toISOString()
        };
        this.socket.emit(RobotCommandFromInterface.UpdateControllerCode, message);
    }

    notifyRobotsToCommunicate(): void {
        const message: NotifyRobotsToCommunicate = {
            command: "P2P",
            timestamp: new Date().toISOString()
        };
        this.socket.emit(RobotCommandFromInterface.NotifyRobotsToCommunicate, message);
    }

    findFurthestRobot(relativePoint: { x: number; y: number }): void {
        const message: FindFurthestRobot = {
            command: "find_furthest",
            relative_point: relativePoint,
            timestamp: new Date().toISOString()
        };
        this.socket.emit(RobotCommandFromInterface.FindFurthestRobot, message);
    }

    identifyRobot(target: "1" | "2"): void {
        const message: IdentifyRobot = {
            command: "identify_robot",
            target
        };
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
