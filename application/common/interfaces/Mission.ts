import { LogMessage } from '@common/interfaces/LogMessage';
import { OccupancyGrid } from '@common/interfaces/LiveMap';

export interface Mission {
    id: string;
    logs: LogMessage[];
    map: OccupancyGrid[];
}