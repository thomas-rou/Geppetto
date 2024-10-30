import { Injectable } from '@angular/core';
import { SocketHandlerService } from './socket-handler/socket-handler.service';
import { ClientCommand } from '@common/enums/ClientCommand';
import { LogMessage } from '@common/interfaces/LogMessage';

@Injectable({
    providedIn: 'root',
})
export class MissionService {
    constructor(private socketService: SocketHandlerService) {
        this.connect();
    }
    public missionLogs: LogMessage[] = [];
    connect() {
        if (!this.socketService.isSocketAlive()) {
            this.socketService.connect();
            this.handleMissionLogs();
        }
    }

    handleMissionLogs() {
        this.socketService.on(ClientCommand.MissionLogs, (logs: LogMessage[]) => {
            this.missionLogs = logs;
        });
    }

    getMissionLogs() {
        this.socketService.send(ClientCommand.MissionLogs);
    }
}
