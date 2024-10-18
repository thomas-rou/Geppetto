import { Component, HostListener } from '@angular/core';
import { CommonModule } from '@angular/common';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { StartMissionPopupComponent } from '@app/components/start-mission-popup/start-mission-popup.component';
import { RobotId } from '@common/enums/RobotId';

@Component({
    selector: 'app-control-panel',
    standalone: true,
    templateUrl: './control-panel.component.html',
    styleUrls: ['./control-panel.component.scss'],
    imports: [CommonModule, StartMissionPopupComponent],
})
export class ControlPanelComponent {
    showPopup: boolean = false;

    constructor(
        private robotService: RobotCommunicationService,
    ) {}

    @HostListener('window:keydown', ['$event'])
    handleKeyDown(event: KeyboardEvent) {
        if (event.key === 'Escape') {
            this.showPopup = false;
        }
    }

    startMission() {
        if (this.robotService.verifySocketConnection()) {
            this.showPopup = true;
        }
    }

    onMissionStart() {
        this.showPopup = false;
        this.robotService.startMission();
    }

    onCancel() {
        this.showPopup = false;
    }

    stopMission() {
        if (this.robotService.verifySocketConnection()) {
            try {
                this.robotService.endMission();
            } catch (error) {
                console.error('Error stopping mission', error);
            }
        }
    }

    identifyRobot(target: RobotId) {
        if (this.robotService.verifySocketConnection()) {
            try {
                this.robotService.identifyRobot(target);
            } catch (error) {
                console.error('Error identifying robot', error);
            }
        }
    }

    returnHome() {
        if (this.robotService.verifySocketConnection()) {
            try {
                this.robotService.returnToBase();
            } catch (error) {
                console.error('Error identifying robot', error);
            }
        }
    }

    updateSoftware() {
        if (this.robotService.verifySocketConnection()) {
            try {
                this.robotService.updateControllerCode('new code here');
            } catch (error) {
                console.error('Error identifying robot', error);
            }
        }
    }
}
