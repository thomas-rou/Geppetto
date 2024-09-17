export interface StartMission {
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
}
