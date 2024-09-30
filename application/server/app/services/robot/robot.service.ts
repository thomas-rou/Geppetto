import { Injectable, Logger } from '@nestjs/common';
import { WebSocket } from 'ws';
import { EndMissionRequest, RobotRequest, StartMissionRequest, MessageOperation } from '@common/interfaces/request.interface';
import { Command, Operation, Topic, TopicType } from '@common/enums/SocketsEvents';

@Injectable()
export class RobotService {
    private readonly logger: Logger = new Logger(RobotService.name);
    private robotIp: string;
    private ws: WebSocket;
    constructor(robotIp: string) {
        this.robotIp = robotIp;
        this.connect();
    }

    async connect() {
        return new Promise<void>((resolve, reject) => {
            this.ws = new WebSocket(`ws://${this.robotIp}:${process.env.ROS_BRIDGING_PORT}`);

            this.ws.onopen = () => {
                this.logger.log(`Connection established to robot ${this.robotIp}`);
                resolve();
            };

            this.ws.onerror = (error) => {
                this.logger.error(`WebSocket error: ${error.message}`);
            };

            this.ws.onclose = () => {
                this.logger.log(`WebSocket connection closed`);
            };
        });
    }

    async subscribeToTopic(topicName: Topic, topicType: TopicType) {
        try {
            if (this.ws.readyState === WebSocket.CLOSED) {
                await this.connect();
            }
            const subscribeMessage: MessageOperation = {
                op: Operation.subscribe,
                topic: topicName,
                type: topicType,
            };
            this.ws.send(JSON.stringify(subscribeMessage));
            this.logger.log(`Subscription to topic ${topicName} of robot ${this.robotIp}`);
        } catch (error) {
            this.logger.error(`Error connecting to robot ${this.robotIp}`);
        }
    }

    async publishToTopic(topicName: Topic, topicType: TopicType, message: RobotRequest) {
        try {
            if (this.ws.readyState === WebSocket.CLOSED) {
                await this.connect();
            }
            const publishMessage: MessageOperation = {
                op: Operation.publish,
                topic: topicName,
                type: topicType,
                msg: message,
            };
            this.ws.send(JSON.stringify(publishMessage));
            this.logger.log(`Publish message to topic ${topicName} of robot ${this.robotIp}:`);
        } catch (error) {
            this.logger.error(`Error connecting to robot ${this.robotIp}`);
        }
    }

    startMission() {
        this.publishToTopic(Topic.start_mission, TopicType.start_mission, {
            command: Command.StartMission,
            mission_details: {
                orientation: 0.0,
                position: {
                    x: 0.0,
                    y: 0.0,
                },
            },
            timestamp: new Date().toISOString(),
        } as StartMissionRequest);
    }

    stopMission() {
        this.publishToTopic(Topic.stop_mission, TopicType.stop_mission, {
            command: Command.EndMission,
            timestamp: new Date().toISOString(),
        } as EndMissionRequest);
    }

    identify(target: '1' | '2') {
        var topicCommand;
        if (target === '1') {
            topicCommand = Topic.identify_command1;
        } else if (target === '2') {
            topicCommand = Topic.identify_command2;
        }
        this.publishToTopic(topicCommand, TopicType.identify_robot, {
            command: Command.Identify,
            timestamp: new Date().toISOString(),
        } as RobotRequest);
    }
}
