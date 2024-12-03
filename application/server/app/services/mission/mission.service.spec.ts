import { Test, TestingModule } from '@nestjs/testing';
import { getModelToken } from '@nestjs/mongoose';
import { MissionService } from './mission.service';
import { MissionDocument } from '@app/model/database/Mission';
import { Model } from 'mongoose';
import { LogMessage } from '@common/interfaces/LogMessage';
import { MissionType } from '@common/enums/MissionType';
import { Mission } from '@app/model/database/Mission';

const mockMission: Mission = {
  id: 'test-id',
  logs: [
    { source: 'system', log_type: 'info', date: new Date().toISOString(), message: 'Log entry 1' },
    { source: 'system', log_type: 'info', date: new Date().toISOString(), message: 'Log entry 2' },
  ],
  map: [],
  missionType: MissionType.GAZEBO_SIMULATION,
  missionDuration: '2h 30m',
  traveledDistance: 1500,
  robots: ['robot1', 'robot2'],
};

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

    await service.createMission(mockMission);
    expect(model.create).toHaveBeenCalledWith(mockMission);
    expect(service.missionId).toBe(mockMission.id);
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

describe('MissionService Error Handling', () => {
  let service: MissionService;
  let model: Model<MissionDocument>;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        MissionService,
        {
          provide: getModelToken('Mission'),
          useValue: {
            create: jest.fn().mockRejectedValue(new Error('Create Error')),
            updateOne: jest.fn().mockRejectedValue(new Error('Update Error')),
            findOne: jest.fn().mockRejectedValue(new Error('Find One Error')),
            find: jest.fn().mockRejectedValue(new Error('Find Error')),
            deleteOne: jest.fn().mockRejectedValue(new Error('Delete One Error')),
            deleteMany: jest.fn().mockRejectedValue(new Error('Delete Many Error')),
          },
        },
      ],
    }).compile();

    service = module.get<MissionService>(MissionService);
    model = module.get<Model<MissionDocument>>(getModelToken('Mission'));
  });

  it('should handle error when creating a mission', async () => {
    await expect(service.createMission(mockMission)).rejects.toEqual('Failed to create mission test-id');
  });

  it('should handle error when adding a log to a mission', async () => {
    const log: LogMessage = {
      message: 'test log',
      log_type: 'string',
      date: 'string',
      source: 'ss'
    };
    await expect(service.addLogToMission('test-id', log)).rejects.toEqual('Failed to add log to mission test-id');
  });

  it('should handle error when getting a mission', async () => {
    await expect(service.getMission('test-id')).rejects.toThrow('Find One Error');
  });
  
  it('should handle error when getting all missions', async () => {
    await expect(service.getAllMissions()).rejects.toThrow('Find Error');
  });

  it('should handle error when deleting a mission', async () => {
    await expect(service.deleteMission('test-id')).rejects.toEqual('Failed to delete mission test-id');
  });

  it('should handle error when deleting all missions', async () => {
    await expect(service.deleteAllMissions()).rejects.toEqual('Failed to delete all missions');
  });
});