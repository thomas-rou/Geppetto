import { Logger } from '@nestjs/common';
import { Test, TestingModule } from '@nestjs/testing';
import { DateService } from '@app/services/date/date.service';
import { ExampleService } from './example.service';

describe('ExampleService', () => {
    let service: ExampleService;

    beforeEach(async () => {
        const module: TestingModule = await Test.createTestingModule({
            providers: [ExampleService, DateService, Logger],
        }).compile();

        service = module.get<ExampleService>(ExampleService);
    });

    it('should be defined', () => {
        expect(service).toBeDefined();
    });
});
