export interface MapMetaData {
    width: number;
    height: number;
    resolution: number;
    origin: { x: number; y: number; z: number };
}

export interface OccupancyGrid {
    header: {
        stamp: { sec: number; nsec: number };
        frame_id: string;
    };
    info: MapMetaData;
    data: Int8Array;
}

