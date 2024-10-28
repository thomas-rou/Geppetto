import { Component, OnDestroy, OnInit, HostListener } from '@angular/core';
import { CommonModule } from '@angular/common';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { LogsService } from '@app/services/logs/logs.service';
import { Subscription } from 'rxjs';
import { StartMissionPopupComponent } from '@app/components/start-mission-popup/start-mission-popup.component';
import { RobotId } from '@common/enums/RobotId';
import { collapseExpandAnimation } from 'src/assets/CollapseExpand';

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
        this.logsService.triggerClearLogs();
    }

    onSimulationMissionStart() {
        this.showPopup = false;
        this.robotService.startMissionGazebo();
        this.logsService.triggerClearLogs();
    }

    onCancel() {
        this.showPopup = false;
    }

    stopMission() {
        if (this.verifySocketConnection()) {
            try {
                this.robotService.endMission();
            } catch (error) {
                console.error('Error stopping mission', error);
            }
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
