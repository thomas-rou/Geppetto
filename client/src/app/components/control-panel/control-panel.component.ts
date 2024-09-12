import { Component } from '@angular/core';
import { RobotCommunicationService } from '@app/services/robot-communication.service';

@Component({
    selector: 'app-control-panel',
    standalone: true,
    templateUrl: './control-panel.component.html',
    styleUrls: ['./control-panel.component.scss'],
})
export class ControlPanelComponent {
    constructor(private robotService: RobotCommunicationService) {}

    startMission() {
        this.robotService.startMission('north', { x: 0, y: 0 }).subscribe(
            (response) => alert('Mission started!'),
            (error) => console.error('Error starting mission', error),
        );
    }

    stopMission() {
        this.robotService.endMission().subscribe(
            (response) => alert('Mission stopped!'),
            (error) => console.error('Error stopping mission', error),
        );
    }

    returnHome() {
        this.robotService.returnToBase().subscribe(
            (response) => alert('Returning home!'),
            (error) => console.error('Error returning home', error),
        );
    }

    updateSoftware() {
        this.robotService.updateControllerCode('new code here').subscribe(
            (response) => alert('Software updated!'),
            (error) => console.error('Error updating software', error),
        );
    }
}
