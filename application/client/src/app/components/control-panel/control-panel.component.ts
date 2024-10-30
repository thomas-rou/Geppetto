import { Component, OnDestroy, OnInit, HostListener } from '@angular/core';
import { CommonModule } from '@angular/common';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { LogsService } from '@app/services/logs/logs.service';
import { Subscription } from 'rxjs';
import { StartMissionPopupComponent } from '@app/components/start-mission-popup/start-mission-popup.component';
import { RobotId } from '@common/enums/RobotId';
import { collapseExpandAnimation } from 'src/assets/CollapseExpand';
import { MissionService } from '@app/services/mission/mission.service';
import { MissionType } from '@app/enums/ClientCommand';

@Component({
    selector: 'app-control-panel',
    standalone: true,
    templateUrl: './control-panel.component.html',
    styleUrls: ['./control-panel.component.scss'],
    imports: [CommonModule, StartMissionPopupComponent],
    animations: [collapseExpandAnimation]
})
export class ControlPanelComponent implements OnInit, OnDestroy {
    private subscriptions: Subscription[] = [];
    private socketConnected: boolean = false;

    showPopup: boolean = false;
    isCollapsed = false;

    constructor(
        private robotService: RobotCommunicationService,
        private logsService: LogsService,
        private missionService: MissionService
    ) {}

    ngOnInit(): void {
        this.subscriptions.push(
            this.robotService.onConnectionStatus().subscribe((isConnected) => {
                this.socketConnected = isConnected;
            }),
        );
    }

    @HostListener('window:keydown', ['$event'])
    handleKeyDown(event: KeyboardEvent) {
        if (event.key === 'Escape') {
            this.showPopup = false;
        }
    }

    toggleCollapse() {
        this.isCollapsed = !this.isCollapsed;
    }

    verifySocketConnection() {
        if (this.socketConnected) return true;
        else return false;
    }

    startMission() {
        if (this.verifySocketConnection()) {
            this.showPopup = true;
        }
    }

    onPhysicalMissionStart() {
        this.showPopup = false;
        this.robotService.startMissionRobot();
        this.missionService.setMissionType(MissionType.Physical);
        this.logsService.triggerClearLogs();
    }

    onSimulationMissionStart() {
        this.showPopup = false;
        this.robotService.startMissionGazebo();
        this.missionService.setMissionType(MissionType.Simulation);
        this.logsService.triggerClearLogs();
    }

    onCancel() {
        this.showPopup = false;
    }

    stopMission() {
        switch (this.missionService.getMissionType()) {
            case MissionType.Physical:
                this.robotService.endMissionRobot();
                break;
            case MissionType.Simulation:
                this.robotService.endMissionGazebo();
                break;
        }
    }

    identifyRobot(target: RobotId) {
        if (this.verifySocketConnection()) {
            try {
                this.robotService.identifyRobot(target);
            } catch (error) {
                console.error('Error identifying robot', error);
            }
        }
    }

    returnHome() {
        if (this.verifySocketConnection()) {
            try {
                this.robotService.returnToBase();
            } catch (error) {
                console.error('Error identifying robot', error);
            }
        }
    }

    updateSoftware() {
        if (this.verifySocketConnection()) {
            try {
                this.robotService.updateControllerCode('new code here');
            } catch (error) {
                console.error('Error identifying robot', error);
            }
        }
    }

    ngOnDestroy(): void {
        this.subscriptions.forEach((sub) => sub.unsubscribe());
    }
}
