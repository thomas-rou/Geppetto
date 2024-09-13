import { Injectable, Logger } from '@nestjs/common';
import { WebSocket } from 'ws';

@Injectable()
export class RobotService {
    private readonly DELAY_TIME:number = 5000;
    private readonly CONNECTION_TIMEOUT:number = 60000;
    private intervalId:NodeJS.Timeout;
    private readonly logger:Logger = new Logger(RobotService.name);
    private robotIp: string;
    private ws: WebSocket;
    private isSocketOpen: boolean = false;
    constructor(robotIp) {
        this.robotIp = robotIp;
        this.connect();
    }

    connect() {
        this.ws = new WebSocket(`ws://${this.robotIp}:${process.env.ROS_BRIDGING_PORT}`);

        this.ws.onopen = () => {
            this.logger.log(`Connection established to robot ${this.robotIp}`);
            this.isSocketOpen = true;
        };

        this.ws.onmessage = (message) => {
            const data = JSON.parse(message.data);
            this.logger.debug(`Message reçu du robot à ${this.robotIp}`, data);
        };

        this.ws.onerror = (error) => {
            this.logger.log(`Error occurred on robot  ${this.robotIp}: ${error.message}`);
            this.isSocketOpen = false;
            this.intervalId = setTimeout(() => {
                this.connect();
            }, this.DELAY_TIME);
            setTimeout(() => {
                clearInterval(this.intervalId);
            }, this.CONNECTION_TIMEOUT);
        };

        this.ws.onclose = () => {
            this.logger.log(`Connection to robot ${this.robotIp} closed`);
        };
    }

    subscribeToTopic(topicName: string, messageType: string) {
        const subscribeMessage = {
            op: 'subscribe',
            topic: topicName,
            type: messageType,
        };
        this.ws.send(JSON.stringify(subscribeMessage));
        this.logger.log(`Subscription to topic ${topicName} of robot ${this.robotIp}`);
    }

    publishToTopic(topicName: string, message: any) {
        const publishMessage = {
            op: 'publish',
            topic: topicName,
            msg: message,
        };
        this.ws.send(JSON.stringify(publishMessage));
        this.logger.log(`Publish message to topic ${topicName} of robot ${this.robotIp}:`, message);
    }

    startMission() {
        this.publishToTopic('/start_mission', {});
    }
    stopMission() {
        this.publishToTopic('/stop_mission', {});
    }
}
