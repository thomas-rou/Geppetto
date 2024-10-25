import { Injectable, Logger } from '@nestjs/common';
import { WebSocket } from 'ws';
import { MessageOperation } from '@common/interfaces/MessageOperation';
import { StartMission } from '@common/interfaces/StartMission';
import { EndMission } from '@common/interfaces/EndMission';
import { RobotCommand } from '@common/enums/RobotCommand';
import { Operation } from '@common/enums/Operation';
import { Topic } from '@common/enums/Topic';
import { TopicType } from '@common/enums/TopicType';
import { RobotId } from '@common/enums/RobotId';
import { BasicCommand } from '@common/interfaces/BasicCommand';

@Injectable()
export class RobotService {
    private readonly logger: Logger = new Logger(RobotService.name);
    private _robotIp: string;
    private _robotNumber: RobotId;
    private ws: WebSocket;

    constructor(robotIp: string, robotNb: RobotId) {
        this._robotIp = robotIp;
        this._robotNumber = robotNb;
        this.connect();
    }

    async connect() {
        return new Promise<void>((resolve, reject) => {
            this.ws = new WebSocket(`ws://${this._robotIp}:${process.env.ROS_BRIDGING_PORT}`);

            this.ws.onopen = () => {
                this.logger.log(`Connection established to robot ${this._robotIp}`);
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

    async subscribeToTopic(topicName: Topic, topicType: TopicType, handleIncomingMessage: (message) => void) {
        try {
            if (this.ws.readyState !== WebSocket.OPEN) {
                await this.connect();
            }
            const subscribeMessage: MessageOperation = {
                op: Operation.subscribe,
                topic: topicName,
                type: topicType,
            };
            this.ws.send(JSON.stringify(subscribeMessage));
            this.logger.log(`Subscription to topic ${topicName} of robot ${this._robotIp}`);
            this.ws.addEventListener('message', (event) => {
            try {
                const messageData = JSON.parse(event.data);
                if (messageData.topic === topicName) {
                    handleIncomingMessage(messageData);
                }
            } catch (error) {
                this.logger.error(`Error processing message from topic ${topicName}: ${error}`);
            }
        });
        } catch (error) {
            this.logger.error(`Error connecting to robot ${this._robotIp}`);
        }
    }

    async publishToTopic(topicName: Topic, topicType: TopicType, message: BasicCommand) {
        try {
            if (this.ws.readyState !== WebSocket.OPEN) {
                await this.connect();
            }
            const publishMessage: MessageOperation = {
                op: Operation.publish,
                topic: topicName,
                type: topicType,
                msg: message,
            };
            this.ws.send(JSON.stringify(publishMessage));
            this.logger.log(`Publish message to topic ${topicName} of robot ${this._robotIp}:`);
        } catch (error) {
            this.logger.error(`Error connecting to robot ${this._robotIp}`);
        }
    }

    startMission() {
        this.publishToTopic(Topic.start_mission, TopicType.start_mission, {
            command: RobotCommand.StartMission,
            mission_details: {
                orientation1: 0.0,
                position1: {
                    x: 0.0,
                    y: 0.0,
                },
                orientation2: 0.0,
                position2: {
                    x: 0.0,
                    y: 0.0,
                },
            },
            timestamp: new Date().toISOString(),
        } as StartMission);
    }

    stopMission() {
        this.publishToTopic(Topic.stop_mission, TopicType.stop_mission, {
            command: RobotCommand.EndMission,
            timestamp: new Date().toISOString(),
        } as EndMission);
    }

    identify() {
        var topicCommand;
        if (this._robotNumber == RobotId.robot1) {
            topicCommand = Topic.identify_command1;
        } else if (this._robotNumber == RobotId.robot2) {
            topicCommand = Topic.identify_command2;
        }
        this.publishToTopic(topicCommand, TopicType.identify_robot, {
            command: RobotCommand.IdentifyRobot,
            timestamp: new Date().toISOString(),
        } as BasicCommand);
    }
}
