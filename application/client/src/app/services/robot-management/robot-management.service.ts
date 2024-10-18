import { Injectable } from '@angular/core';
import { Robot } from '@app/classes/robot/robot';
import { RobotId } from '@common/enums/RobotId';
@Injectable({
    providedIn: 'root',
})
export class RobotManagementService {
    robot1: Robot = new Robot(RobotId.robot1, 'Robot 1', 'EN ATTENTE', 100, { x: 0, y: 0 }, 0.0);
    robot2: Robot = new Robot(RobotId.robot2, 'Robot 2', 'EN ATTENTE', 100, { x: 0, y: 0 }, 0.0);
}