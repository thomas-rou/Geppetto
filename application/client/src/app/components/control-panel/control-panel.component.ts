import { Component, OnDestroy, OnInit } from '@angular/core';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { Subscription } from 'rxjs';
import { NotificationService } from '../../services/notification.service';

@Component({
    selector: 'app-control-panel',
    standalone: true,
    templateUrl: './control-panel.component.html',
    styleUrls: ['./control-panel.component.scss'],
})
export class ControlPanelComponent implements OnInit, OnDestroy {
    private subscriptions: Subscription[] = [];
    private socketConnected: boolean = false;

    constructor(private robotService: RobotCommunicationService, private notificationService: NotificationService) {}

    ngOnInit(): void {
        this.subscriptions.push(
                this.robotService.onMissionStatus().subscribe((message) => {
                    this.notificationService.sendNotification(message);
                }),
                this.robotService.onRobotIdentification().subscribe((message) => {
                    this.notificationService.sendNotification(message);
                }),
                this.robotService.onCommandError().subscribe((message) => {
                    this.notificationService.sendNotification(`Error: ${message}`);
                }),
                
                this.robotService.onConnectionStatus().subscribe((isConnected) => {
                if (isConnected)
                    console.log('WebSocket is connected')
                else
                    console.log('WebSocket is disconnected');
                this.socketConnected = isConnected;
            })
        );
    }

    verifySocketConnection() {
        if(this.socketConnected)
            return true;
        else 
            this.notificationService.sendNotification("No socket connection has been established");
        return false;
    }

    startMission() {
        if(this.verifySocketConnection()) {
            try {
                this.robotService.startMission(0, { x: 0, y: 0 });
            } catch (error) {
                console.error('Error starting mission', error);
            }
        }
    }

    stopMission() {
        if(this.verifySocketConnection()) {
            try {
                this.robotService.endMission();
            } catch (error) {
                console.error('Error stopping mission', error);
            }
        }
    }

    identifyRobot(target: '1' | '2') {
        if(this.verifySocketConnection()) {
            try {
                this.robotService.identifyRobot(target);
            } catch (error) {
                console.error('Error identifying robot', error);
            } 
        }
    }

    returnHome() {
        if(this.verifySocketConnection()) {
            try {
                this.robotService.returnToBase;
            } catch (error) {
                console.error('Error identifying robot', error);
            }
        }
    }

    updateSoftware() {
        if(this.verifySocketConnection()) {
            try {
                this.robotService.updateControllerCode('new code here');
            } catch (error) {
                console.error('Error identifying robot', error);
            } 
        }
    }

    ngOnDestroy(): void {
        this.subscriptions.forEach((sub) => sub.unsubscribe());
    }
}
