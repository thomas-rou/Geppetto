import { LogMessage } from '@common/interfaces/LogMessage';
import { Prop, Schema, SchemaFactory } from '@nestjs/mongoose';
import { ApiProperty } from '@nestjs/swagger';
import { Document } from 'mongoose';

export type MissionDocument = Mission & Document;

@Schema()
export class Mission {
    @ApiProperty()
    @Prop({ required: true })
    id: string;

    @ApiProperty()
    @Prop({ required: true })
    logs: LogMessage[];
}

export const MissionSchema = SchemaFactory.createForClass(Mission);
