import { Injectable } from '@angular/core';
import { Robot } from '@app/classes/robot/robot';
import { RobotStatus } from '@app/enums/robot-status';

@Injectable({
    providedIn: 'root',
})
export class RobotManagementService {
    robot1: Robot = new Robot('1', 'Robot 1', RobotStatus.Idle, 100, { x: 0, y: 0 }, 0.0);
    robot2: Robot = new Robot('2', 'Robot 2', RobotStatus.Idle, 100, { x: 0, y: 0 }, 0.0);
}
