import { LogMessage } from '@common/interfaces/LogMessage';
import { Prop, Schema, SchemaFactory } from '@nestjs/mongoose';
import { ApiProperty } from '@nestjs/swagger';
import { Document } from 'mongoose';
import { OccupancyGrid } from '@common/interfaces/LiveMap';

export type MissionDocument = Mission & Document;

@Schema()
export class Mission {
    @ApiProperty()
    @Prop({ required: true })
    id: string;

    @ApiProperty()
    @Prop({ required: true })
    logs: LogMessage[];

    @ApiProperty()
    @Prop({ required: false })
    map: OccupancyGrid
}

export const MissionSchema = SchemaFactory.createForClass(Mission);
