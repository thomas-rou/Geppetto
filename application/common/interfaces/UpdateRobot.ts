export interface UpdateRobot {
    command: string;
    identifier: string;
    status: string;
    position: {
        x: number;
        y: number;
    };
    timestamp: string;
}
