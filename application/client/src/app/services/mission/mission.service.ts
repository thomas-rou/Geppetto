import { Injectable } from '@angular/core';
import { MissionType } from '@app/enums/MissionType';
import { SocketHandlerService } from '@app/services/socket-handler/socket-handler.service';
import { ClientCommand } from '@common/enums/ClientCommand';
import { Mission } from '@common/interfaces/Mission';

@Injectable({
    providedIn: 'root',
})
export class MissionService {
    private missionType: MissionType;

    public missions: Mission[] = [];

    constructor(private socketService: SocketHandlerService) {
        this.connect();
    }

    async connect() {
        if (!this.socketService.isSocketAlive()) {
            this.socketService.connect();
            await this.handleMissionLogs();
        }
    }

    async handleMissionLogs(): Promise<void> {
        return new Promise((resolve) => {
            this.socketService.on(ClientCommand.MissionLogs, (missions: Mission[]) => {
                this.missions = missions;
                resolve();
            });
        });
    }

    getMissionLogs() {
        this.socketService.send(ClientCommand.MissionLogs);
    }

    getMissionType() {
        return this.missionType;
    }

    setMissionType(newMissionType: MissionType) {
        this.missionType = newMissionType;
    }
}
