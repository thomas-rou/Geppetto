import { Mission, MissionDocument } from '@app/model/database/Mission';
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
            this.logger.log(`Adding log to mission ${missionId}`);
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
}
