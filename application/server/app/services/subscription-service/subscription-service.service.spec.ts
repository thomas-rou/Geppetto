import { Test, TestingModule } from '@nestjs/testing';
import { SubscriptionServiceService } from './subscription-service.service';

describe('SubscriptionServiceService', () => {
  let service: SubscriptionServiceService;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [SubscriptionServiceService],
    }).compile();

    service = module.get<SubscriptionServiceService>(SubscriptionServiceService);
  });

  it('should be defined', () => {
    expect(service).toBeDefined();
  });
});
