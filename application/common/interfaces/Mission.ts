import { LogMessage } from '@common/interfaces/LogMessage';

export interface Mission {
    id: string;
    logs: LogMessage[];
}