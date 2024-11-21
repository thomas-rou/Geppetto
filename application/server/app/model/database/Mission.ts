import { LogMessage } from '@common/interfaces/LogMessage';
import { Prop, Schema, SchemaFactory } from '@nestjs/mongoose';
import { ApiProperty } from '@nestjs/swagger';
import { Document } from 'mongoose';

export type MissionDocument = Mission & Document;

enum MissionType {
    PHYSICAL_ROBOTS = 'Physical Robots',
    GAZEBO_SIMULATION = 'Gazebo Simulation',
}

@Schema()
export class Mission {
    @ApiProperty()
    @Prop({ required: true })
    id: string;

    @ApiProperty()
    @Prop({ required: true })
    logs: LogMessage[];

    @ApiProperty()
    @Prop({ required: true })
    missionType: MissionType;

    @ApiProperty()
    @Prop({ required: true })
    missionDuration: number;

    @ApiProperty()
    @Prop({ required: true })
    traveledDistance: number;

    @ApiProperty({ description: 'List of robots participating in the mission' })
    @Prop({ required: true, type: [String] })
    robots: string[];

}

export const MissionSchema = SchemaFactory.createForClass(Mission);
