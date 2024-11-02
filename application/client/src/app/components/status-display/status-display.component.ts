import { Component } from '@angular/core';
import { RobotManagementService } from '@app/services/robot-management/robot-management.service';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { RobotId } from '@common/enums/RobotId';
import { collapseExpandAnimation } from 'src/assets/CollapseExpand';

@Component({
    selector: 'app-status-display',
    standalone: true,
    imports: [],
    templateUrl: './status-display.component.html',
    styleUrl: './status-display.component.scss',
    animations: [collapseExpandAnimation],
})
export class StatusDisplayComponent {
    isCollapsed = false;

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

    toggleCollapse() {
        this.isCollapsed = !this.isCollapsed;
    }

    identifyRobot(target: RobotId) {
        this.robotService.identifyRobot(target);
    }
}
