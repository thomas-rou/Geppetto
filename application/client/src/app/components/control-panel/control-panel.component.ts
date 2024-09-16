import { Component, OnDestroy, OnInit } from '@angular/core';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { Subscription } from 'rxjs';

@Component({
    selector: 'app-control-panel',
    standalone: true,
    templateUrl: './control-panel.component.html',
    styleUrls: ['./control-panel.component.scss'],
})
export class ControlPanelComponent implements OnInit, OnDestroy {
    private subscriptions: Subscription[] = [];

    constructor(private robotService: RobotCommunicationService) {}

    ngOnInit(): void {
        this.subscriptions.push(
            this.robotService.onMissionStatus().subscribe((message) => {
                alert(message);
            }),
            this.robotService.onRobotIdentification().subscribe((message) => {
                alert(message);
            }),
            this.robotService.onCommandError().subscribe((message) => {
                alert(`Error: ${message}`);
            }),
        );
    }

    startMission() {
        try {
            this.robotService.startMission(0, { x: 0, y: 0 });
        } catch (error) {
            console.error('Error starting mission', error);
        }
    }

    stopMission() {
        try {
            this.robotService.endMission();
        } catch (error) {
            console.error('Error stopping mission', error);
        }
    }

    identifyRobot(target: '1' | '2') {
        try {
            this.robotService.identifyRobot(target);
        } catch (error) {
            console.error('Error identifying robot', error);
        }
    }

    returnHome() {
        try {
            this.robotService.returnToBase;
        } catch (error) {
            console.error('Error identifying robot', error);
        }
    }

    updateSoftware() {
        try {
            this.robotService.updateControllerCode('new code here');
        } catch (error) {
            console.error('Error identifying robot', error);
        }
    }

    ngOnDestroy(): void {
        this.subscriptions.forEach((sub) => sub.unsubscribe());
    }
}
