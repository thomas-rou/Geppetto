import { ComponentFixture, TestBed } from '@angular/core/testing';
import { ControlPanelComponent } from './control-panel.component';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { NotificationService } from '@app/services/notification/notification.service';
import { of } from 'rxjs';
import { StartMissionPopupComponent } from "@app/components/start-mission-popup/start-mission-popup.component";
import { CommonModule } from '@angular/common';

describe('ControlPanelComponent', () => {
    let component: ControlPanelComponent;
    let fixture: ComponentFixture<ControlPanelComponent>;
    let robotService: jasmine.SpyObj<RobotCommunicationService>;
    let notificationService: jasmine.SpyObj<NotificationService>;

    beforeEach(async () => {
        const robotServiceSpy = jasmine.createSpyObj('RobotCommunicationService', [
            'onMissionStatus', 'onRobotIdentification', 'onCommandError', 'onConnectionStatus', 'startMission', 'endMission', 'returnToBase', 'updateControllerCode'
        ]);
        const notificationServiceSpy = jasmine.createSpyObj('NotificationService', ['sendNotification']);

        await TestBed.configureTestingModule({
            imports: [CommonModule, StartMissionPopupComponent],
            providers: [
                { provide: RobotCommunicationService, useValue: robotServiceSpy },
                { provide: NotificationService, useValue: notificationServiceSpy }
            ]
        }).compileComponents();

        fixture = TestBed.createComponent(ControlPanelComponent);
        component = fixture.componentInstance;
        robotService = TestBed.inject(RobotCommunicationService) as jasmine.SpyObj<RobotCommunicationService>;
        notificationService = TestBed.inject(NotificationService) as jasmine.SpyObj<NotificationService>;

        robotService.onMissionStatus.and.returnValue(of('Mission Status'));
        robotService.onRobotIdentification.and.returnValue(of('Robot Identification'));
        robotService.onCommandError.and.returnValue(of('Command Error'));
        robotService.onConnectionStatus.and.returnValue(of(true));
    });

    it('should create', () => {
        expect(component).toBeTruthy();
    });

    it('should initialize subscriptions on ngOnInit', () => {
        component.ngOnInit();
        expect(robotService.onMissionStatus).toHaveBeenCalled();
        expect(robotService.onRobotIdentification).toHaveBeenCalled();
        expect(robotService.onCommandError).toHaveBeenCalled();
        expect(robotService.onConnectionStatus).toHaveBeenCalled();
    });

    it('should handle keydown event and close popup on Escape key', () => {
        component.showPopup = true;
        const event = new KeyboardEvent('keydown', { key: 'Escape' });
        component.handleKeyDown(event);
        expect(component.showPopup).toBe(false);
    });

    it('should verify socket connection', () => {
        component['socketConnected'] = true;
        expect(component.verifySocketConnection()).toBe(true);

        component['socketConnected'] = false;
        expect(component.verifySocketConnection()).toBe(false);
        expect(notificationService.sendNotification).toHaveBeenCalledWith('No socket connection has been established');
    });

    it('should start mission if socket is connected', () => {
        spyOn(component, 'verifySocketConnection').and.returnValue(true);
        component.startMission();
        expect(component.showPopup).toBe(true);
    });

    it('should not start mission if socket is not connected', () => {
        spyOn(component, 'verifySocketConnection').and.returnValue(false);
        component.startMission();
        expect(component.showPopup).toBe(false);
    });

    it('should handle mission start', () => {
        component.onMissionStart();
        expect(component.showPopup).toBe(false);
        expect(robotService.startMission).toHaveBeenCalled();
    });

    it('should stop mission if socket is connected', () => {
        spyOn(component, 'verifySocketConnection').and.returnValue(true);
        component.stopMission();
        expect(robotService.endMission).toHaveBeenCalled();
    });

    it('should handle error when stopping mission', () => {
        spyOn(component, 'verifySocketConnection').and.returnValue(true);
        robotService.endMission.and.throwError('Error stopping mission');
        spyOn(console, 'error');
        component.stopMission();
        expect(console.error).toHaveBeenCalledWith('Error stopping mission', jasmine.any(Error));
    });

    it('should return home if socket is connected', () => {
        spyOn(component, 'verifySocketConnection').and.returnValue(true);
        component.returnHome();
        expect(robotService.returnToBase).toHaveBeenCalled();
    });

    it('should handle error when returning home', () => {
        spyOn(component, 'verifySocketConnection').and.returnValue(true);
        robotService.returnToBase.and.throwError('Error identifying robot');
        spyOn(console, 'error');
        component.returnHome();
        expect(console.error).toHaveBeenCalledWith('Error identifying robot', jasmine.any(Error));
    });

    it('should update software if socket is connected', () => {
        spyOn(component, 'verifySocketConnection').and.returnValue(true);
        component.updateSoftware();
        expect(robotService.updateControllerCode).toHaveBeenCalledWith('new code here');
    });

    it('should handle error when updating software', () => {
        spyOn(component, 'verifySocketConnection').and.returnValue(true);
        robotService.updateControllerCode.and.throwError('Error identifying robot');
        spyOn(console, 'error');
        component.updateSoftware();
        expect(console.error).toHaveBeenCalledWith('Error identifying robot', jasmine.any(Error));
    });

    it('should unsubscribe from all subscriptions on ngOnDestroy', () => {
        component.ngOnInit();
        component.ngOnDestroy();
        component['subscriptions'].forEach(sub => {
            expect(sub.closed).toBe(true);
        });
    });
    
    it('should log "WebSocket is disconnected" when the connection status is false', () => {
        spyOn(console, 'log');
        robotService.onConnectionStatus.and.returnValue(of(false));
        component.ngOnInit();
        expect(console.log).toHaveBeenCalledWith('WebSocket is disconnected');
    });
});