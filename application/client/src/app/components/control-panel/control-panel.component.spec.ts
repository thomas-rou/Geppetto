import { TestBed } from '@angular/core/testing';
import { ControlPanelComponent } from './control-panel.component';
import { RobotCommunicationService } from '@app/services/robot-communication.service';
import { of, throwError } from 'rxjs';

describe('ControlPanelComponent', () => {
    let component: ControlPanelComponent;
    let robotService: jasmine.SpyObj<RobotCommunicationService>;

    beforeEach(() => {
        const robotServiceSpy = jasmine.createSpyObj('RobotCommunicationService', [
            'startMission',
            'endMission',
            'returnToBase',
            'updateControllerCode',
        ]);

        TestBed.configureTestingModule({
            imports: [ControlPanelComponent],
            providers: [{ provide: RobotCommunicationService, useValue: robotServiceSpy }],
        }).compileComponents();

        const fixture = TestBed.createComponent(ControlPanelComponent);
        component = fixture.componentInstance;
        robotService = TestBed.inject(RobotCommunicationService) as jasmine.SpyObj<RobotCommunicationService>;
    });

    it('should create', () => {
        expect(component).toBeTruthy();
    });

    it('should start mission successfully', () => {
        robotService.startMission.and.returnValue(of({}));
        spyOn(window, 'alert');

        component.startMission();

        expect(robotService.startMission).toHaveBeenCalledWith('north', { x: 0, y: 0 });
        expect(window.alert).toHaveBeenCalledWith('Mission started!');
    });

    it('should handle error when starting mission', () => {
        robotService.startMission.and.returnValue(throwError('error'));
        spyOn(console, 'error');

        component.startMission();

        expect(robotService.startMission).toHaveBeenCalledWith('north', { x: 0, y: 0 });
        expect(console.error).toHaveBeenCalledWith('Error starting mission', 'error');
    });

    it('should stop mission successfully', () => {
        robotService.endMission.and.returnValue(of({}));
        spyOn(window, 'alert');

        component.stopMission();

        expect(robotService.endMission).toHaveBeenCalled();
        expect(window.alert).toHaveBeenCalledWith('Mission stopped!');
    });

    it('should handle error when stopping mission', () => {
        robotService.endMission.and.returnValue(throwError('error'));
        spyOn(console, 'error');

        component.stopMission();

        expect(robotService.endMission).toHaveBeenCalled();
        expect(console.error).toHaveBeenCalledWith('Error stopping mission', 'error');
    });

    it('should return home successfully', () => {
        robotService.returnToBase.and.returnValue(of({}));
        spyOn(window, 'alert');

        component.returnHome();

        expect(robotService.returnToBase).toHaveBeenCalled();
        expect(window.alert).toHaveBeenCalledWith('Returning home!');
    });

    it('should handle error when returning home', () => {
        robotService.returnToBase.and.returnValue(throwError('error'));
        spyOn(console, 'error');

        component.returnHome();

        expect(robotService.returnToBase).toHaveBeenCalled();
        expect(console.error).toHaveBeenCalledWith('Error returning home', 'error');
    });

    it('should update software successfully', () => {
        robotService.updateControllerCode.and.returnValue(of({}));
        spyOn(window, 'alert');

        component.updateSoftware();

        expect(robotService.updateControllerCode).toHaveBeenCalledWith('new code here');
        expect(window.alert).toHaveBeenCalledWith('Software updated!');
    });

    it('should handle error when updating software', () => {
        robotService.updateControllerCode.and.returnValue(throwError('error'));
        spyOn(console, 'error');

        component.updateSoftware();

        expect(robotService.updateControllerCode).toHaveBeenCalledWith('new code here');
        expect(console.error).toHaveBeenCalledWith('Error updating software', 'error');
    });
});
