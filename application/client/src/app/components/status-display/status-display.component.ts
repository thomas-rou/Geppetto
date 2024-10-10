import { Component } from '@angular/core';
import { RobotManagementService } from '@app/services/robot-management/robot-management.service';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { RobotId } from '@common/enums/RobotId';

@Component({
    selector: 'app-status-display',
    standalone: true,
    imports: [],
    templateUrl: './status-display.component.html',
    styleUrl: './status-display.component.scss',
})
export class StatusDisplayComponent {
    constructor(
        private robotService: RobotCommunicationService,
        private robotManagementService: RobotManagementService,
    ) {}

    get robot1() {
        return this.robotManagementService.robot1;
    }
    get robot2() {
        return this.robotManagementService.robot2;
    }

    identifyRobot(target: RobotId) {
        this.robotService.identifyRobot(target);
    }
}
