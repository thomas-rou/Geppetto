import { Injectable } from '@angular/core';
import { Robot } from '@app/classes/robot/robot';
import { RobotStatus } from '@app/enums/robot-status';
import { RobotId } from '@common/enums/SocketsEvents';
@Injectable({
    providedIn: 'root',
})
export class RobotManagementService {
    robot1: Robot = new Robot(RobotId.robot1 , 'Robot 1', RobotStatus.Idle, 100, { x: 0, y: 0 }, 0.0);
    robot2: Robot = new Robot(RobotId.robot2, 'Robot 2', RobotStatus.Idle, 100, { x: 0, y: 0 }, 0.0);
}
