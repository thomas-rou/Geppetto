import { Component } from '@angular/core';
import { Robot } from '@app/classes/robot';
import { RobotStatus } from '@app/enums/robot-status.enum';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';

@Component({
    selector: 'app-status-display',
    standalone: true,
    imports: [],
    templateUrl: './status-display.component.html',
    styleUrl: './status-display.component.scss',
})
export class StatusDisplayComponent {
    constructor(private robotService: RobotCommunicationService) {}
    robot1: Robot = new Robot('Robot 1', RobotStatus.Idle, 100, { x: 0, y: 0 }, 0.0);
    robot2: Robot = new Robot('Robot 2', RobotStatus.Idle, 100, { x: 0, y: 0 }, 0.0);
    identifyRobot1() { this.robotService.identifyRobot("1"); }
    identifyRobot2() { this.robotService.identifyRobot("2"); }
}
