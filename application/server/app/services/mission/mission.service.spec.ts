import { Test, TestingModule } from '@nestjs/testing';
import { getModelToken } from '@nestjs/mongoose';
import { MissionService } from './mission.service';
import { MissionDocument } from '@app/model/database/Mission';
import { Model } from 'mongoose';
import { LogMessage } from '@common/interfaces/LogMessage';

const mission = { id: 'test-id', logs: [] };

describe('MissionService', () => {
  let service: MissionService;
  let model: Model<MissionDocument>;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        MissionService,
        {
          provide: getModelToken('Mission'),
          useValue: {
            create: jest.fn(),
            updateOne: jest.fn(),
            findOne: jest.fn(),
            find: jest.fn(),
            deleteOne: jest.fn(),
            deleteMany: jest.fn(),
          },
        },
      ],
    }).compile();

    service = module.get<MissionService>(MissionService);
    model = module.get<Model<MissionDocument>>(getModelToken('Mission'));
  });

  it('should create a mission', async () => {
    await service.createMission(mission);
    expect(model.create).toHaveBeenCalledWith(mission);
    expect(service.missionId).toBe(mission.id);
  });

  it('should add a log to a mission', async () => {
    const missionId = 'test-id';
    const log: LogMessage = {
      message: 'test log',
      log_type: 'string',
      date: 'string',
      source: 'ss'
    };
    await service.addLogToMission(missionId, log);
    expect(model.updateOne).toHaveBeenCalledWith({ id: missionId }, { $push: { logs: log } });
  });

  it('should get a mission', async () => {
    const missionId = 'test-id';
    await service.getMission(missionId);
    expect(model.findOne).toHaveBeenCalledWith({ id: missionId });
  });

  it('should get all missions', async () => {
    await service.getAllMissions();
    expect(model.find).toHaveBeenCalled();
  });

  it('should delete a mission', async () => {
    const missionId = 'test-id';
    await service.deleteMission(missionId);
    expect(model.deleteOne).toHaveBeenCalledWith({ id: missionId });
  });

  it('should delete all missions', async () => {
    await service.deleteAllMissions();
    expect(model.deleteMany).toHaveBeenCalledWith({});
  });
});