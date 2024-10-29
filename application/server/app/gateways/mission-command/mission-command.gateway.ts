import { RobotCommand } from '@common/enums/RobotCommand';
import { EndMission } from '@common/interfaces/EndMission';
import { StartMission } from '@common/interfaces/StartMission';
import { IdentifyRobot } from '@common/interfaces/IdentifyRobot';
import { RobotId } from '@common/enums/RobotId';
import { Injectable } from '@nestjs/common';
import { SubscribeMessage, WebSocketGateway, WebSocketServer } from '@nestjs/websockets';
import { Server, Socket } from 'socket.io';
import { SubscriptionServiceService } from '@app/services/subscription-service/subscription-service.service';
import { LogService } from '@app/services/log/log.service';
import { LogType } from '@common/enums/LogType';

@Injectable()
@WebSocketGateway()
export class MissionCommandGateway {
    @WebSocketServer()
    server: Server;
    private logger : LogService;
    private controllingClient: Socket | null = null;

    constructor(private subscriptionService: SubscriptionServiceService) {
        this.logger = new LogService(this.server);
    }

    handleDisconnect(client: Socket) {
        if (this.controllingClient === client) {
            this.logger.logToClient(LogType.INFO,'Controlling client disconnected, allowing new controller');
            this.controllingClient = null;
        }
    }

    verifyPermissionToControl(client: Socket): boolean {
        if (this.controllingClient === null) {
            this.controllingClient = client;
            this.logger.server = this.server;
            this.logger.logToClient(LogType.INFO,`Client ${client.id} is now controlling the robots`);
            return true;
        } else if (this.controllingClient !== client) {
            this.logger.logToClient(LogType.INFO,`Client ${client.id} sent a control command, but another client is already controlling`);
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
                this.logger.logToClient(LogType.INFO,`${commands[command].log} for robots command received from client`);
                await this.subscriptionService.robot1[commands[command].method]();
                await this.subscriptionService.robot2[commands[command].method]();
                await this.subscriptionService.subscribeToTopicRobots(this);
                this.server.emit('missionStatus', `${commands[command].successMessage} for robots`);
            } else if (targets.includes(RobotId.gazebo)) {
                this.logger.logToClient(LogType.INFO,`${commands[command].log} for simulation command received from client`);
                await this.subscriptionService.gazebo[commands[command].method]();
                await this.subscriptionService.subscribeToTopicGazebo(this);
                this.server.emit('missionStatus', `${commands[command].successMessage} for the simulation`);
            } else {
                this.logger.logToClient(LogType.ERROR,'Invalid mission command');
                this.server.emit('commandError', `Invalid ${command} mission command`);
            }
        } catch (e) {
            this.logger.logToClient(LogType.ERROR,`Error in ${command} MissionRobots: ${e.message}`);
            this.server.emit('commandError', `${e.message} please try again`);
        }
    }

    @SubscribeMessage(RobotCommand.StartMission)
    async startMissionRobots(client: Socket, payload: StartMission) {
        await this.handleMissionCommand(client, payload, 'start');
    }

    @SubscribeMessage(RobotCommand.EndMission)
    async stopMissionFromRobots(client: Socket, payload: EndMission) {
        await this.handleMissionCommand(client, payload, 'stop');
    }

    @SubscribeMessage(RobotCommand.IdentifyRobot)
    async identifyRobot(client: Socket, payload: IdentifyRobot) {
        if (!this.verifyPermissionToControl(client)) {
            client.emit('commandError', 'The system is already being controlled');
            return;
        }

        try {
            switch (payload.target) {
                case RobotId.robot1:
                    this.logger.logToClient(LogType.INFO,'Identify robot 1 command received from client');
                    await this.subscriptionService.robot1.identify();
                    this.server.emit('robotIdentification', 'Robot 1 was identified');
                    break;
                case RobotId.robot2:
                    this.logger.logToClient(LogType.INFO,'Identify robot 2 command received from client');
                    await this.subscriptionService.robot2.identify();
                    this.server.emit('robotIdentification', 'Robot 2 was identified');
                    break;
                default:
                    this.logger.logToClient(LogType.ERROR,'Invalid identify robot command');
                    this.server.emit('commandError', 'Invalid identify robot command');
                    return;
            }
        } catch (e) {
            this.logger.logToClient(LogType.ERROR,'Error in identifyRobot: ' + e.message);
            this.server.emit('commandError', `${e.message} please try again`);
        }
    }
}
