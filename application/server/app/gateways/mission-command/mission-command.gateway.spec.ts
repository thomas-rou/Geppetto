import { Test, TestingModule } from '@nestjs/testing';
import { MissionCommandGateway } from './mission-command.gateway';

describe('MissionCommandGateway', () => {
    let gateway: MissionCommandGateway;

    beforeEach(async () => {
        const module: TestingModule = await Test.createTestingModule({
            providers: [MissionCommandGateway],
        }).compile();

        gateway = module.get<MissionCommandGateway>(MissionCommandGateway);
    });

    it('should be defined', () => {
        expect(gateway).toBeDefined();
    });
});
