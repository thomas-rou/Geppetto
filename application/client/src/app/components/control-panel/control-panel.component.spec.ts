import { ComponentFixture, TestBed } from '@angular/core/testing';
import { ControlPanelComponent } from './control-panel.component';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { LogsService } from '@app/services/logs/logs.service';
import { MissionService } from '@app/services/mission/mission.service';
import { of } from 'rxjs';
import { MissionType } from '@app/enums/MissionType';
import { RobotId } from '@common/enums/RobotId';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';

describe('ControlPanelComponent', () => {
    let component: ControlPanelComponent;
    let fixture: ComponentFixture<ControlPanelComponent>;
    let robotServiceMock: any;
    let logsServiceMock: any;
    let missionServiceMock: any;

    beforeEach(async () => {
        robotServiceMock = {
            onConnectionStatus: jasmine.createSpy('onConnectionStatus').and.returnValue(of(true)),
            startMissionRobot: jasmine.createSpy('startMissionRobot'),
            startMissionGazebo: jasmine.createSpy('startMissionGazebo'),
            endMissionRobot: jasmine.createSpy('endMissionRobot'),
            endMissionGazebo: jasmine.createSpy('endMissionGazebo'),
            identifyRobot: jasmine.createSpy('identifyRobot'),
            returnToBase: jasmine.createSpy('returnToBase'),
            updateControllerCode: jasmine.createSpy('updateControllerCode'),
        };

        logsServiceMock = {
            triggerClearLogs: jasmine.createSpy('triggerClearLogs'),
        };

        missionServiceMock = {
            setMissionType: jasmine.createSpy('setMissionType'),
            getMissionType: jasmine.createSpy('getMissionType').and.returnValue(MissionType.Physical),
        };

        await TestBed.configureTestingModule({
            imports: [BrowserAnimationsModule],
            providers: [
                { provide: RobotCommunicationService, useValue: robotServiceMock },
                { provide: LogsService, useValue: logsServiceMock },
                { provide: MissionService, useValue: missionServiceMock },
            ],
        }).compileComponents();
    });

    beforeEach(() => {
        fixture = TestBed.createComponent(ControlPanelComponent);
        component = fixture.componentInstance;
        fixture.detectChanges();
    });

    it('should create', () => {
        expect(component).toBeTruthy();
    });

    it('should initialize and subscribe to connection status', () => {
        component.ngOnInit();
        expect(robotServiceMock.onConnectionStatus).toHaveBeenCalled();
    });

    it('should handle key down event and close popup on Escape key', () => {
        component.showPopup = true;
        const event = new KeyboardEvent('keydown', { key: 'Escape' });
        component.handleKeyDown(event);
        expect(component.showPopup).toBeFalse();
    });

    it('should toggle collapse state', () => {
        component.isCollapsed = false;
        component.toggleCollapse();
        expect(component.isCollapsed).toBeTrue();
    });

    it('should verify socket connection', () => {
        component['socketConnected'] = true;
        expect(component.verifySocketConnection()).toBeTrue();
    });

    it('should start mission and show popup if socket is connected', () => {
        component['socketConnected'] = true;
        component.startMission();
        expect(component.showPopup).toBeTrue();
    });

    it('should start physical mission', () => {
        component.onPhysicalMissionStart();
        expect(robotServiceMock.startMissionRobot).toHaveBeenCalled();
        expect(missionServiceMock.setMissionType).toHaveBeenCalledWith(MissionType.Physical);
        expect(logsServiceMock.triggerClearLogs).toHaveBeenCalled();
    });

    it('should start simulation mission', () => {
        component.onSimulationMissionStart();
        expect(robotServiceMock.startMissionGazebo).toHaveBeenCalled();
        expect(missionServiceMock.setMissionType).toHaveBeenCalledWith(MissionType.Simulation);
        expect(logsServiceMock.triggerClearLogs).toHaveBeenCalled();
    });

    it('should cancel mission and hide popup', () => {
        component.showPopup = true;
        component.onCancel();
        expect(component.showPopup).toBeFalse();
    });

    it('should stop mission based on mission type', () => {
        component.stopMission();
        expect(robotServiceMock.endMissionRobot).toHaveBeenCalled();
    });

    it('should identify robot if socket is connected', () => {
        component['socketConnected'] = true;
        component.identifyRobot(RobotId.robot1);
        expect(robotServiceMock.identifyRobot).toHaveBeenCalledWith(RobotId.robot1);
    });

    it('should return home if socket is connected', () => {
        component['socketConnected'] = true;
        component.returnHome();
        expect(robotServiceMock.returnToBase).toHaveBeenCalled();
    });

    it('should update software if socket is connected', () => {
        component['socketConnected'] = true;
        component.updateSoftware();
        expect(robotServiceMock.updateControllerCode).toHaveBeenCalledWith('new code here');
    });

    it('should unsubscribe from all subscriptions on destroy', () => {
        const subscription = jasmine.createSpyObj('Subscription', ['unsubscribe']);
        component['subscriptions'] = [subscription];
        component.ngOnDestroy();
        expect(subscription.unsubscribe).toHaveBeenCalled();
    });

    it('should return false when socket is not connected', () => {
        component['socketConnected'] = false;
        expect(component.verifySocketConnection()).toBeFalse();
    });

    it('should stop simulation mission', () => {
        missionServiceMock.getMissionType.and.returnValue(MissionType.Simulation);
        component.stopMission();
        expect(robotServiceMock.endMissionGazebo).toHaveBeenCalled();
    });

    it('should handle error when identifying robot', () => {
        component['socketConnected'] = true;
        robotServiceMock.identifyRobot.and.throwError('Error identifying robot');
        spyOn(console, 'error');
        component.identifyRobot(RobotId.robot1);
        expect(console.error).toHaveBeenCalledWith('Error identifying robot', jasmine.any(Error));
    });

    it('should handle error when returning home', () => {
        component['socketConnected'] = true;
        robotServiceMock.returnToBase.and.throwError('Error identifying robot');
        spyOn(console, 'error');
        component.returnHome();
        expect(console.error).toHaveBeenCalledWith('Error identifying robot', jasmine.any(Error));
    });

    it('should handle error when updating software', () => {
        component['socketConnected'] = true;
        robotServiceMock.updateControllerCode.and.throwError('Error identifying robot');
        spyOn(console, 'error');
        component.updateSoftware();
        expect(console.error).toHaveBeenCalledWith('Error identifying robot', jasmine.any(Error));
    });
});
