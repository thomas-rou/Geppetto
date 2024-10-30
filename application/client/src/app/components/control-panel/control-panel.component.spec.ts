import { ComponentFixture, TestBed } from '@angular/core/testing';
import { ControlPanelComponent } from './control-panel.component';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { of } from 'rxjs';
import { StartMissionPopupComponent } from '@app/components/start-mission-popup/start-mission-popup.component';
import { CommonModule } from '@angular/common';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';

describe('ControlPanelComponent', () => {
    let component: ControlPanelComponent;
    let fixture: ComponentFixture<ControlPanelComponent>;
    let robotService: jasmine.SpyObj<RobotCommunicationService>;

    beforeEach(async () => {
        const robotServiceSpy = jasmine.createSpyObj('RobotCommunicationService', [
            'onMissionStatus',
            'onRobotIdentification',
            'onCommandError',
            'onConnectionStatus',
            'startMissionRobot',
            'startMissionGazebo',
            'endMissionRobot',
            'endMissionGazebo',
            'returnToBase',
            'updateControllerCode',
        ]);
        
        await TestBed.configureTestingModule({
            imports: [CommonModule, BrowserAnimationsModule, StartMissionPopupComponent],
            providers: [
                { provide: RobotCommunicationService, useValue: robotServiceSpy },
            ],
        }).compileComponents();

        fixture = TestBed.createComponent(ControlPanelComponent);
        component = fixture.componentInstance;
        robotService = TestBed.inject(RobotCommunicationService) as jasmine.SpyObj<RobotCommunicationService>;

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
        component.onPhysicalMissionStart();
        expect(component.showPopup).toBe(false);
        expect(robotService.startMissionRobot).toHaveBeenCalled();
    });

    it('should stop robot mission if socket is connected', () => {
        spyOn(component, 'verifySocketConnection').and.returnValue(true);
        component.stopMission();
        expect(robotService.endMissionRobot).toHaveBeenCalled();
    });

    it('should stop simulation mission if socket is connected', () => {
        spyOn(component, 'verifySocketConnection').and.returnValue(true);
        component.stopMission();
        expect(robotService.endMissionGazebo).toHaveBeenCalled();
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
        component['subscriptions'].forEach((sub) => {
            expect(sub.closed).toBe(true);
        });
    });

    it('should log "WebSocket is disconnected" when the connection status is false', () => {
        spyOn(console, 'log');
        robotService.onConnectionStatus.and.returnValue(of(false));
        component.ngOnInit();
        expect(console.log).toHaveBeenCalledWith('WebSocket is disconnected');
    });

    it('should set showPopup to false when onCancel is called', () => {
        component.showPopup = true;
        component.onCancel();
        expect(component.showPopup).toBe(false);
    });
});
