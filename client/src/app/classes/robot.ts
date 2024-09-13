import { RobotStatus } from '@app/enums/robot-status';

type Coords = {
    x: number;
    y: number;
};

export class Robot {
    private _name: string;
    private _status: RobotStatus;
    private _battery: number;
    private _position: Coords;
    private _orientation: number;

    constructor(name: string, status: RobotStatus, battery: number, position: Coords, orientation: number) {
        this._name = name;
        this._status = status;
        this._battery = battery;
        this._position = position;
        this._orientation = orientation;
    }

    get name(): string {
        return this._name;
    }

    get status(): RobotStatus {
        return this._status;
    }

    get battery(): number {
        return this._battery;
    }

    get position(): Coords {
        return this._position;
    }

    get orientation(): number {
        return this._orientation;
    }

    set name(name: string) {
        this._name = name;
    }

    set status(status: RobotStatus) {
        this._status = status;
    }

    set battery(battery: number) {
        this._battery = battery;
    }

    set position(position: Coords) {
        this._position = position;
    }
}
