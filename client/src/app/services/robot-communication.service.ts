import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';

@Injectable({
    providedIn: 'root',
})
export class RobotCommunicationService {
    private apiUrl = 'http://localhost:3000/api';

    constructor(private http: HttpClient) {}

    startMission(orientation: string, position: { x: number; y: number }): Observable<unknown> {
        const body = {
            command: 'start_mission',
            mission_details: {
                orientation,
                position,
            },
            timestamp: new Date().toISOString(),
        };
        return this.http.post(this.apiUrl, body);
    }

    endMission(): Observable<unknown> {
        const body = {
            command: 'end_mission',
            timestamp: new Date().toISOString(),
        };
        return this.http.post(this.apiUrl, body);
    }

    updateRobot(name: string, status: string, position: { x: number; y: number }): Observable<unknown> {
        const body = {
            command: 'update',
            name,
            status,
            position,
            timestamp: new Date().toISOString(),
        };
        return this.http.post(this.apiUrl, body);
    }

    returnToBase(): Observable<unknown> {
        const body = {
            command: 'return_to_base',
            timestamp: new Date().toISOString(),
        };
        return this.http.post(this.apiUrl, body);
    }

    updateControllerCode(newCode: string): Observable<unknown> {
        const body = {
            command: 'update_controller_code',
            code: newCode,
            timestamp: new Date().toISOString(),
        };
        return this.http.post(this.apiUrl, body);
    }

    notifyRobotsToCommunicate(): Observable<unknown> {
        const body = {
            command: 'P2P',
            timestamp: new Date().toISOString(),
        };
        return this.http.post(this.apiUrl, body);
    }

    findFurthestRobot(relativePoint: { x: number; y: number }): Observable<unknown> {
        const body = {
            command: 'find_furthest',
            relative_point: relativePoint,
            timestamp: new Date().toISOString(),
        };
        return this.http.post(this.apiUrl, body);
    }
}
