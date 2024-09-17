import { DateController } from '@app/controllers/date/date.controller';
import { DateService } from '@app/services/date/date.service';
import { Test, TestingModule } from '@nestjs/testing';
import { SinonStubbedInstance, createStubInstance } from 'sinon';

describe('DateController', () => {
    let controller: DateController;
    let dateService: SinonStubbedInstance<DateService>;

    beforeEach(async () => {
        dateService = createStubInstance(DateService);
        const module: TestingModule = await Test.createTestingModule({
            controllers: [DateController],
            providers: [
                {
                    provide: DateService,
                    useValue: dateService,
                },
            ],
        }).compile();

        controller = module.get<DateController>(DateController);
    });

    it('should be defined', () => {
        expect(controller).toBeDefined();
    });
});
