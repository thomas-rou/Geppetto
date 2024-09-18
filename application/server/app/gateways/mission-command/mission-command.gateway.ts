import {RobotCommandFromInterface } from '@app/../../common/enums/SocketsEvents';
import { EndMission } from '@app/../../common/interfaces/EndMission';
import { StartMission } from '@app/../../common/interfaces/StartMission';
import { IdentifyRobot } from '@app/../../common/interfaces/IdentifyRobot';
import { RobotService } from '@app/services/robot/robot.service';
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
        this.robot1 =  new RobotService(process.env.ROBOT1_IP);
        this.robot2 =   new RobotService(process.env.ROBOT2_IP);
        this.gazebo = new RobotService(process.env.GAZEBO_IP);
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
        } 
        if (this.controllingClient !== client) {
            this.logger.log(`Client ${client.id} send a control command, but another client is already controlling`);
            client.emit('commandError', 'Another client is controlling the robots');
            return false;
        }
        return true;
    }

    @SubscribeMessage(RobotCommandFromInterface.StartMission)
    startMissionRobots(client: Socket, payload: StartMission ) {
        //Checking if command received from controlling is valid
        if(this.verifyPermissionToControl(client)) {
            //Executing command
            if(payload.target === 'robot') {
                this.logger.log('Start mission for robots command received from client');
                this.robot1.startMission();
                this.robot2.startMission();
                this.server.emit('missionStatus', 'Mission started for robots');
            } else if (payload.target === 'sim') {
                this.logger.log('Start mission for simulation command received from client');
                this.gazebo.startMission();
                this.server.emit('missionStatus', 'Mission started for the simulation');
            } else {
                this.logger.error('Invalid mission start command');
                this.server.emit('commandError', 'Invalid start mission command');
            }
        } else {
            client.emit('commandError', 'The system is already being controlled')
        }

    }
    
    @SubscribeMessage(RobotCommandFromInterface.EndMission)
    stopMissionFromRobots(client: Socket, payload: EndMission) {
        //Checking if command received from controlling is valid
        if(this.verifyPermissionToControl(client)) {
            //Executing command
            if(payload.target === 'robot') {
                this.logger.log('Stop mission for robots command received from client');
                this.robot1.stopMission();
                this.robot2.stopMission();
                this.server.emit('missionStatus', 'Mission stopped for robots');
            } else if (payload.target === 'sim') {
                this.logger.log('Stop mission for simulation command received from client');
                this.gazebo.stopMission();
                this.server.emit('missionStatus', 'Mission stopped for the simulation');
            } else {
                this.logger.error('Invalid mission start command');
                this.server.emit('commandError', 'Invalid start mission command');
            }
        } else {
            client.emit('commandError', 'The system is already being controlled')
        }
    }

    @SubscribeMessage(RobotCommandFromInterface.IdentifyRobot)
    identifyRobot(client: Socket, payload: IdentifyRobot) {
        //Checking if command received from controlling is valid
        if(this.verifyPermissionToControl(client)) {
            //Executing command
            if(payload.target === "1") {
                this.logger.log('Identify robot 1 command received from client');
                this.robot1.identify(payload.target);
            } else if(payload.target === "2") {
                this.logger.log('Identify robot 2 command received from client');
                this.robot2.identify(payload.target);
            }
            this.server.emit('robotIdentification', 'Robot was identified');
        } else {
            client.emit('commandError', 'The system is already being controlled')
        }
    } 
}
