export interface StartMissionRequest {
  command: 'start_mission';
  mission_details: {
      orientation: string;
      position: { x: number; y: number };
  };
  timestamp: string;
}

export interface EndMissionRequest {
  command: 'end_mission';
  timestamp: string;
}

export interface UpdateRobotRequest {
  command: 'update';
  name: string;
  status: string;
  position: { x: number; y: number };
  timestamp: string;
}

export interface ReturnToBaseRequest {
  command: 'return_to_base';
  timestamp: string;
}

export interface UpdateControllerCodeRequest {
  command: 'update_controller_code';
  code: string;
  timestamp: string;
}

export interface NotifyRobotsToCommunicateRequest {
  command: 'P2P';
  timestamp: string;
}

export interface FindFurthestRobotRequest {
  command: 'find_furthest';
  relative_point: { x: number; y: number };
  timestamp: string;
}