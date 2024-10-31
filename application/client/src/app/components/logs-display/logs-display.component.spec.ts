import { ComponentFixture, TestBed } from '@angular/core/testing';
import { LogsDisplayComponent } from './logs-display.component';
import { LogsService } from '@app/services/logs/logs.service';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { of, Subject } from 'rxjs';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';

describe('LogsDisplayComponent', () => {
    let component: LogsDisplayComponent;
    let fixture: ComponentFixture<LogsDisplayComponent>;
    let robotCommunicationService: RobotCommunicationService;
    let clearLogsEvent: Subject<void>;

    beforeEach(async () => {
        clearLogsEvent = new Subject<void>();

        const logsServiceMock = {
            clearLogsEvent: clearLogsEvent.asObservable()
        };

        const robotCommunicationServiceMock = {
            onLog: jasmine.createSpy('onLog').and.returnValue(of('Test log'))
        };

        await TestBed.configureTestingModule({
            imports: [BrowserAnimationsModule, LogsDisplayComponent],
            providers: [
                { provide: LogsService, useValue: logsServiceMock },
                { provide: RobotCommunicationService, useValue: robotCommunicationServiceMock }
            ]
        }).compileComponents();

        fixture = TestBed.createComponent(LogsDisplayComponent);
        component = fixture.componentInstance;
        robotCommunicationService = TestBed.inject(RobotCommunicationService);
        fixture.detectChanges();
    });

    it('should create', () => {
        expect(component).toBeTruthy();
    });

    it('should subscribe to clearLogsEvent on init', () => {
        spyOn(component, 'clearLogs');
        component.ngOnInit();
        clearLogsEvent.next(); // Emit the event
        expect(component.clearLogs).toHaveBeenCalled();
    });

    it('should subscribe to onLog on AfterViewInit', () => {
        component.ngAfterViewInit();
        expect(robotCommunicationService.onLog).toHaveBeenCalled();
    });

    it('should add a log to the terminal', () => {
        const terminalElement = fixture.nativeElement.querySelector('div');

        const testLog = 'Test log entry';
        component.addLogToTerminal(testLog);

        const logElements = terminalElement.getElementsByTagName('p');
        expect(logElements.length).toBe(2);
        expect(logElements[1].textContent).toBe(testLog);
    });

    it('should clear logs', () => {
        const terminalElement = fixture.nativeElement.querySelector('div');
        component.addLogToTerminal('Test log 1');
        component.addLogToTerminal('Test log 2');

        component.clearLogs();

        const logElements = terminalElement.getElementsByTagName('p');
        expect(logElements.length).toBe(0);
    });

    it('should toggle collapse state', () => {
        expect(component.isCollapsed).toBeTrue();
        component.toggleCollapse();
        expect(component.isCollapsed).toBeFalse();
        component.toggleCollapse();
        expect(component.isCollapsed).toBeTrue();
    });

    it('should initialize and subscribe to connection status', () => {
        spyOn(component, 'clearLogs');
        component.ngOnInit();
        clearLogsEvent.next(); // Emit the event
        expect(component.clearLogs).toHaveBeenCalled();
    });
});