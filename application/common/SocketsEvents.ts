export enum RobotCommandFromInterface {
    StartMission = 'start_mission',
    EndMission = 'end_mission',
    IdentifyRobot = 'identify_robot',
    UpdateRobot = 'update',
    ReturnToBase = 'return_to_base',
    UpdateControllerCode = 'update_controller_code',
    NotifyRobotsToCommunicate = 'initiate_p2p',
    FindFurthestRobot = 'find_furthest'
}

export class StartMission {
    command: string;
    target: string;
    mission_details: {
        orientation: number;
        position: {
            x: number;
            y: number;
        };
    };
    timestamp: string;

    constructor(target: string, orientation: number, x: number, y: number, timestamp: string) {
        this.command = "start_mission";
        this.target = target;
        this.mission_details = {
            orientation,
            position: { x, y }
        };
        this.timestamp = timestamp;
    }
}

export class EndMission {
    command: string;
    target: string;
    timestamp: string;

    constructor(target: string, timestamp: string) {
        this.command = "end_mission";
        this.target = target;
        this.timestamp = timestamp;
    }
}

export class Update {
    command: string;
    identifier: string;
    status: string;
    position: {
        x: number;
        y: number;
    };
    timestamp: string;

    constructor(identifier: string, status: string, x: number, y: number, timestamp: string) {
        this.command = "update";
        this.identifier = identifier;
        this.status = status;
        this.position = { x, y };
        this.timestamp = timestamp;
    }
}

export class ReturnToBase {
    command: string;
    timestamp: string;

    constructor(timestamp: string) {
        this.command = "return_to_base";
        this.timestamp = timestamp;
    }
}

export class UpdateControllerCode {
    command: string;
    code: string;
    timestamp: string;

    constructor(code: string, timestamp: string) {
        this.command = "update_controller_code";
        this.code = code;
        this.timestamp = timestamp;
    }
}

export class P2PCommunication {
    command: string;
    timestamp: string;

    constructor(timestamp: string) {
        this.command = "P2P";
        this.timestamp = timestamp;
    }
}

export class IdentifyRobot {
    command: string;
    target: string;

    constructor(target: "1" | "2") {
        this.command = "identify_robot";
        this.target = target;
    }
}

export class FindFurthestRobot {
    command: string;
    relative_point: {
        x: number;
        y: number;
    };
    timestamp: string;

    constructor(x: number, y: number, timestamp: string) {
        this.command = "find_furthest";
        this.relative_point = { x, y };
        this.timestamp = timestamp;
    }
}