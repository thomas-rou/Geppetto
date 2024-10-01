import { RobotCommand } from '@common/enums/SocketsEvents';
import { EndMission } from '@common/interfaces/EndMission';
import { StartMission } from '@common/interfaces/StartMission';
import { IdentifyRobot } from '@common/interfaces/IdentifyRobot';
import { RobotService } from '@app/services/robot/robot.service';
import { RobotId } from '@common/enums/SocketsEvents';
import { Injectable, Logger } from '@nestjs/common';
import { SubscribeMessage, WebSocketGateway, WebSocketServer } from '@nestjs/websockets';
import { Server, Socket } from 'socket.io';

@Injectable()
@WebSocketGateway()
export class MissionCommandGateway {
    @WebSocketServer()
    server: Server;
    private readonly logger = new Logger(MissionCommandGateway.name);
    private robot1: RobotService;
    private robot2: RobotService;
    private gazebo: RobotService;
    private controllingClient: Socket | null = null;

    constructor() {
        this.robot1 = new RobotService(process.env.ROBOT1_IP, RobotId.robot1);
        this.robot2 = new RobotService(process.env.ROBOT2_IP, RobotId.robot2);
        this.gazebo = new RobotService(process.env.GAZEBO_IP, RobotId.gazebo);
    }

    handleDisconnect(client: Socket) {
        if (this.controllingClient === client) {
            this.logger.log('Controlling client disconnected, allowing new controller');
            this.controllingClient = null;
        }
    }

    verifyPermissionToControl(client: Socket) {
        if (this.controllingClient === null) {
            this.controllingClient = client;
            this.logger.log(`Client ${client.id} is now controlling the robots`);
        } else if (this.controllingClient !== client) {
            this.logger.log(`Client ${client.id} send a control command, but another client is already controlling`);
            client.emit('commandError', 'Another client is controlling the robots');
            return false;
        }
        return true;
    }

    @SubscribeMessage(RobotCommand.StartMission)
    startMissionRobots(client: Socket, payload: StartMission) {
        try {
            if (this.verifyPermissionToControl(client)) {
                if (payload.target === 'robot') {
                    this.logger.log('Start mission for robots command received from client');
                    this.robot1.startMission();
                    this.robot2.startMission();
                    this.server.emit('missionStatus', 'Mission started for robots');
                } else if (payload.target === 'simulation') {
                    this.logger.log('Start mission for simulation command received from client');
                    this.gazebo.startMission();
                    this.server.emit('missionStatus', 'Mission started for the simulation');
                } else {
                    this.logger.error('Invalid mission start command');
                    this.server.emit('commandError', 'Invalid start mission command');
                }
            } else {
                client.emit('commandError', 'The system is already being controlled');
            }
        } catch (e) {
            this.logger.error('Error in startMissionRobots: ' + e.message);
            this.server.emit('commandError', `${e.message} please try again`);
        }
    }

    @SubscribeMessage(RobotCommand.EndMission)
    stopMissionFromRobots(client: Socket, payload: EndMission) {
        try {
            if (this.verifyPermissionToControl(client)) {
                if (payload.target === 'robot') {
                    this.logger.log('Stop mission for robots command received from client');
                    this.robot1.stopMission();
                    this.robot2.stopMission();
                    this.server.emit('missionStatus', 'Mission stopped for robots');
                } else if (payload.target === 'simulation') {
                    this.logger.log('Stop mission for simulation command received from client');
                    this.gazebo.stopMission();
                    this.server.emit('missionStatus', 'Mission stopped for the simulation');
                } else {
                    this.logger.error('Invalid mission start command');
                    this.server.emit('commandError', 'Invalid start mission command');
                }
            } else {
                client.emit('commandError', 'The system is already being controlled');
            }
        } catch (e) {
            this.logger.error('Error in stopMissionFromRobots: ' + e.message);
            this.server.emit('commandError', `${e.message} please try again`);
        }
    }

    @SubscribeMessage(RobotCommand.IdentifyRobot)
    identifyRobot(client: Socket, payload: IdentifyRobot) {
        try {
            if(this.verifyPermissionToControl(client)) {
                if(payload.target === RobotId.robot1) {
                    this.logger.log('Identify robot 1 command received from client');
                    this.robot1.identify();
                } else if(payload.target === RobotId.robot2) {
                    this.logger.log('Identify robot 2 command received from client');
                    this.robot2.identify();
                }
                this.server.emit('robotIdentification', 'Robot was identified');
            }
        } catch (e) {
            this.logger.error('Error in identifyRobot: ' + e.message);
            this.server.emit('commandError', `${e.message} please try again`);
        }
    }
}
