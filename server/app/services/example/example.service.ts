import { DateService } from '@app/services/date/date.service';
import { Injectable, Logger } from '@nestjs/common';

@Injectable()
export class ExampleService {

    constructor(
        private readonly dateService: DateService,
        private readonly logger: Logger,
    ) {
    }
}
