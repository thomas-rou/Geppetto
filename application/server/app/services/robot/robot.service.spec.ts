import { Test, TestingModule } from '@nestjs/testing';
import { RobotService } from './robot.service';
import { Logger } from '@nestjs/common';
import { RobotId } from '@common/enums/RobotId';

describe('RobotService', () => {
  let service: RobotService;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        RobotService,
        { provide: 'robotIp', useValue: '127.0.0.1' },
        { provide: 'robotNb', useValue: RobotId.robot1 },
        { provide: Logger, useValue: new Logger() },
      ],
    }).compile();

    service = module.get<RobotService>(RobotService);
  });

  it('should be defined', () => {
    expect(service).toBeDefined();
  });
});