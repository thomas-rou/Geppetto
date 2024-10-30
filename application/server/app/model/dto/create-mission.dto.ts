import { LogMessage } from '@common/interfaces/LogMessage';
import { ApiProperty } from '@nestjs/swagger';
import { IsString } from 'class-validator';

export class CreateMissionDto {
    @ApiProperty()
    @IsString()
    id: string;

    @ApiProperty()
    logs: LogMessage[];
}