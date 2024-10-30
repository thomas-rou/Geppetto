import { Injectable } from '@angular/core';
import { SocketHandlerService } from '@app/services/socket-handler/socket-handler.service';
import { ClientCommand } from '@common/enums/ClientCommand';
import { LogMessage } from '@common/interfaces/LogMessage';

@Injectable({
    providedIn: 'root',
})
export class MissionService {
    public missionLogs: LogMessage[] = [];

    constructor(private socketService: SocketHandlerService) {
        this.connect();
    }

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
        console.log(this.missionLogs);
    }

    getMissionLogs() {
        this.socketService.send(ClientCommand.MissionLogs);
    }
}
