import { Test, TestingModule } from '@nestjs/testing';
import { SubscriptionServiceService } from './subscription-service.service';
import { MissionService } from '../mission/mission.service';

describe('SubscriptionServiceService', () => {
  let service: SubscriptionServiceService;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        SubscriptionServiceService,
        {
          provide: MissionService,
          useValue: {
            // Mock any methods used in the SubscriptionServiceService
            addLogToMission: jest.fn(),
            missionId: 'test-mission-id',
          },
        },
      ],
    }).compile();

    service = module.get<SubscriptionServiceService>(SubscriptionServiceService);
  });

  it('should be defined', () => {
    expect(service).toBeDefined();
  });
});