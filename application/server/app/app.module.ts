import { Logger, Module } from '@nestjs/common';
import { ConfigModule, ConfigService } from '@nestjs/config';
import { MissionCommandGateway } from '@app/gateways/mission-command/mission-command.gateway';
import { MongooseModule } from '@nestjs/mongoose';

@Module({
    imports: [
        ConfigModule.forRoot({ isGlobal: true }),
        MongooseModule.forRootAsync({
            imports: [ConfigModule],
            inject: [ConfigService],
            useFactory: async (config: ConfigService) => ({
                uri: config.get<string>('DATABASE_CONNECTION_STRING'),
            }),
        }),
    ],
    providers: [MissionCommandGateway, Logger],
})
export class AppModule {}
