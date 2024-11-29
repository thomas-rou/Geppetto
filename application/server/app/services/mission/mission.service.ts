import { Mission, MissionDocument } from '@app/model/database/Mission';
import { OccupancyGrid } from '@common/interfaces/LiveMap';
import { LogMessage } from '@common/interfaces/LogMessage';
import { Injectable, Logger } from '@nestjs/common';
import { InjectModel } from '@nestjs/mongoose';
import { Model } from 'mongoose';

@Injectable()
export class MissionService {
    constructor(@InjectModel(Mission.name) public missionModel: Model<MissionDocument>) {}
    private readonly logger = new Logger(MissionService.name);
    public missionId = '';
    async createMission(mission: Mission = { id: new Date().toISOString().slice(0, -5), logs: [] } as Mission): Promise<void> {
        try {
            this.logger.log(`Creating mission ${mission.id}`);
            await this.missionModel.create(mission);
            this.missionId = mission.id;
        } catch (err) {
            return Promise.reject(`Failed to create mission ${mission.id}`);
        }
    }
    async addLogToMission(missionId: string, log: LogMessage): Promise<void> {
        try {
            await this.missionModel.updateOne({ id: missionId }, { $push: { logs: log } });
        } catch (err) {
            return Promise.reject(`Failed to add log to mission ${missionId}`);
        }
    }
    async getMission(missionId: string): Promise<Mission | null> {
        try {
            this.logger.log(`Getting mission ${missionId}`);
            return this.missionModel.findOne({ id: missionId });
        } catch (err) {
            return Promise.reject(`Failed to get mission ${missionId}`);
        }
    }
    async getAllMissions(): Promise<Mission[]> {
        try {
            this.logger.log(`Getting all missions`);
            return this.missionModel.find();
        } catch (err) {
            return Promise.reject(`Failed to get all missions`);
        }
    }
    async deleteMission(missionId: string): Promise<void> {
        try {
            this.logger.log(`Deleting mission ${missionId}`);
            await this.missionModel.deleteOne({ id: missionId });
        } catch (err) {
            return Promise.reject(`Failed to delete mission ${missionId}`);
        }
    }
    async deleteAllMissions(): Promise<void> {
        try {
            this.logger.log(`Deleting all missions`);
            await this.missionModel.deleteMany({});
        } catch (err) {
            return Promise.reject(`Failed to delete all missions`);
        }
    }
    async isMissionActive(): Promise<boolean> {
        try {
            if (this.missionId) {
                const mission = await this.missionModel.findOne({ id: this.missionId });
                return !!mission;
            }
            return false;
        } catch (err) {
            this.logger.error('Failed to check mission status', err);
            throw new Error('Failed to check mission status');
        }
    }

    async addMapToMission(missionId: string, map: OccupancyGrid[]): Promise<void> {
        try {
            await this.missionModel.updateOne({ id: missionId }, { map });
        } catch (err) {
            return Promise.reject(`Failed to add map to mission ${missionId}`);
        }
    }
    
}
