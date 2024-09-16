import { RobotService } from '@app/services/robot/robot.service';
import { Injectable, Logger } from '@nestjs/common';

@Injectable()
export class MissionCommandGateway {
    private readonly logger = new Logger(MissionCommandGateway.name);
    private simulator = new RobotService(process.env.SIMULATION_IP);
}
