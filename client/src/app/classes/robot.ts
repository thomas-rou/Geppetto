import { RobotStatus } from '@app/enums/robot-status';

export class Robot {
    name: string;
    status: RobotStatus;
    battery: number;
    position: { x: number; y: number };

    constructor(name: string, status: RobotStatus, battery: number, position: { x: number; y: number }) {
        this.name = name;
        this.status = status;
        this.battery = battery;
        this.position = position;
    }
}
