import { Message } from '@app/model/schema/message.schema';
import { ExampleService } from '@app/services/example/example.service';
import { Body, Controller, Get, Post } from '@nestjs/common';
import { ApiCreatedResponse, ApiOkResponse, ApiTags } from '@nestjs/swagger';

@ApiTags('Example')
@Controller('example')
export class ExampleController {
    constructor(private readonly exampleService: ExampleService) {}
}
