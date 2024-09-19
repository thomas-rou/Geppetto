import { Logger, Module } from '@nestjs/common';
import { ConfigModule } from '@nestjs/config';
import { MissionCommandGateway } from '@app/gateways/mission-command/mission-command.gateway';

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
    providers: [MissionCommandGateway, Logger],
})
export class AppModule {}
