import { Injectable } from '@angular/core';
import { MissionType } from '@app/enums/ClientCommand';
import { SocketHandlerService } from '@app/services/socket-handler/socket-handler.service';
import { ClientCommand } from '@common/enums/ClientCommand';
import { LogMessage } from '@common/interfaces/LogMessage';

@Injectable({
    providedIn: 'root',
})
export class MissionService {
    private missionType: MissionType;

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

    getMissionType() {
        return this.missionType;
    }

    setMissionType(newMissionType: MissionType) {
        this.missionType = newMissionType;
    }
}
