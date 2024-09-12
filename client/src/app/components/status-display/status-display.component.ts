import { Component } from '@angular/core';
import { Robot } from '@app/classes/robot';
import { RobotStatus } from '@app/enums/robot-status';

@Component({
    selector: 'app-status-display',
    standalone: true,
    imports: [],
    templateUrl: './status-display.component.html',
    styleUrl: './status-display.component.scss',
})
export class StatusDisplayComponent {
    robot1: Robot = new Robot('Robot 1', RobotStatus.Idle, 100, { x: 0, y: 0 });
    robot2: Robot = new Robot('Robot 2', RobotStatus.Idle, 100, { x: 0, y: 0 });
}
