import { RobotCommand } from '@common/enums/RobotCommand';
import { EndMission } from '@common/interfaces/EndMission';
import { StartMission } from '@common/interfaces/StartMission';
import { IdentifyRobot } from '@common/interfaces/IdentifyRobot';
import { RobotId } from '@common/enums/RobotId';
import { Injectable, Logger } from '@nestjs/common';
import { SubscribeMessage, WebSocketGateway, WebSocketServer } from '@nestjs/websockets';
import { Server, Socket } from 'socket.io';
import { SubscriptionServiceService } from '@app/services/subscription-service/subscription-service.service';

@Injectable()
@WebSocketGateway()
export class MissionCommandGateway {
    @WebSocketServer()
    server: Server;
    private readonly logger = new Logger(MissionCommandGateway.name);
    private controllingClient: Socket | null = null;

    constructor(private subscriptionService: SubscriptionServiceService) {}

    handleDisconnect(client: Socket) {
        if (this.controllingClient === client) {
            this.logger.log('Controlling client disconnected, allowing new controller');
            this.controllingClient = null;
        }
    }

    verifyPermissionToControl(client: Socket): boolean {
        if (this.controllingClient === null) {
            this.controllingClient = client;
            this.logger.log(`Client ${client.id} is now controlling the robots`);
            return true;
        } else if (this.controllingClient !== client) {
            this.logger.log(`Client ${client.id} sent a control command, but another client is already controlling`);
            client.emit('commandError', 'Another client is controlling the robots');
            return false;
        }
        return true;
    }

    private async handleMissionCommand(client: Socket, payload: { target: RobotId[] }, command: 'start' | 'stop') {
        if (!this.verifyPermissionToControl(client)) {
            client.emit('commandError', 'The system is already being controlled');
            return;
        }

        const targets = payload.target;
        const commands = {
            start: {
                log: 'Start mission',
                method: 'startMission',
                successMessage: 'Mission started',
            },
            stop: {
                log: 'Stop mission',
                method: 'stopMission',
                successMessage: 'Mission stopped',
            },
        };

        try {
            if (targets.includes(RobotId.robot1) && targets.includes(RobotId.robot2)) {
                this.logger.log(`${commands[command].log} for robots command received from client`);
                this.subscriptionService.robot1[commands[command].method]();
                this.subscriptionService.robot2[commands[command].method]();
                this.server.emit('missionStatus', `${commands[command].successMessage} for robots`);
            } else if (targets.includes(RobotId.gazebo)) {
                this.logger.log(`${commands[command].log} for simulation command received from client`);
                this.subscriptionService.gazebo[commands[command].method]();
                this.server.emit('missionStatus', `${commands[command].successMessage} for the simulation`);
            } else {
                this.logger.error('Invalid mission command');
                this.server.emit('commandError', `Invalid ${command} mission command`);
            }
            if (command === 'start') await this.subscriptionService.subscribeToTopic(this);
        } catch (e) {
            this.logger.error(`Error in ${command}MissionRobots: ${e.message}`);
            this.server.emit('commandError', `${e.message} please try again`);
        }
    }

    @SubscribeMessage(RobotCommand.StartMission)
    startMissionRobots(client: Socket, payload: StartMission) {
        this.handleMissionCommand(client, payload, 'start');
    }

    @SubscribeMessage(RobotCommand.EndMission)
    stopMissionFromRobots(client: Socket, payload: EndMission) {
        this.handleMissionCommand(client, payload, 'stop');
    }

    @SubscribeMessage(RobotCommand.IdentifyRobot)
    identifyRobot(client: Socket, payload: IdentifyRobot) {
        if (!this.verifyPermissionToControl(client)) {
            client.emit('commandError', 'The system is already being controlled');
            return;
        }

        try {
            switch (payload.target) {
                case RobotId.robot1:
                    this.logger.log('Identify robot 1 command received from client');
                    this.subscriptionService.robot1.identify();
                    this.server.emit('robotIdentification', 'Robot 1 was identified');
                    break;
                case RobotId.robot2:
                    this.logger.log('Identify robot 2 command received from client');
                    this.subscriptionService.robot2.identify();
                    this.server.emit('robotIdentification', 'Robot 2 was identified');
                    break;
                default:
                    this.logger.error('Invalid identify robot command');
                    this.server.emit('commandError', 'Invalid identify robot command');
                    return;
            }
        } catch (e) {
            this.logger.error('Error in identifyRobot: ' + e.message);
            this.server.emit('commandError', `${e.message} please try again`);
        }
    }
}
