import { RobotStatus } from '@app/enums/robot-status';
import { Position } from '@common/types/Position';
import { RobotId } from '@common/enums/SocketsEvents';

export class Robot {
    private _id: RobotId;
    private _name: string;
    private _status: RobotStatus;
    private _battery: number;
    private _position: Position;
    private _orientation: number;

    constructor(id: RobotId, name: string, status: RobotStatus, battery: number, position: Position, orientation: number) {
        this._id = id;
        this._name = name;
        this._status = status;
        this._battery = battery;
        this._position = position;
        this._orientation = orientation;
    }

    get id(): RobotId {
        return this._id;
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

    get position(): Position {
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

    set position(position: Position) {
        this._position = position;
    }

    set orientation(orientation: number) {
        this._orientation = orientation;
    }
}
