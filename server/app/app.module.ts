import { Logger, Module } from '@nestjs/common';
import { ConfigModule, ConfigService } from '@nestjs/config';
import { DateController } from '@app/controllers/date/date.controller';
import { DateService } from '@app/services/date/date.service';
import { ChatGateway } from '@app/gateways/chat/chat.gateway';
import { ExampleService } from '@app/services/example/example.service';
import { ExampleController } from '@app/controllers/example/example.controller';

@Module({
    imports: [
        ConfigModule.forRoot({ isGlobal: true }),
        // MongooseModule.forRootAsync({
        //     imports: [ConfigModule],
        //     inject: [ConfigService],
        //     useFactory: async (config: ConfigService) => ({
        //         uri: config.get<string>('DATABASE_CONNECTION_STRING'), // Loaded from .env
        //     }),
        // }),
        // MongooseModule.forFeature([{ name: Course.name, schema: courseSchema }]),
    ],
    controllers: [DateController, ExampleController],
    providers: [ChatGateway, DateService, ExampleService, Logger],
})
export class AppModule {}
