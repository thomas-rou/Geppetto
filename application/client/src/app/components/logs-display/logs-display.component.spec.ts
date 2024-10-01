import { ComponentFixture, discardPeriodicTasks, fakeAsync, TestBed, tick } from '@angular/core/testing';

import { LogsDisplayComponent } from './logs-display.component';

describe('LogsDisplayComponent', () => {
    let component: LogsDisplayComponent;
    let fixture: ComponentFixture<LogsDisplayComponent>;

    beforeEach(async () => {
        await TestBed.configureTestingModule({
            imports: [LogsDisplayComponent],
        }).compileComponents();

        fixture = TestBed.createComponent(LogsDisplayComponent);
        component = fixture.componentInstance;
        fixture.detectChanges();
    });

    it('should create', () => {
        expect(component).toBeTruthy();
    });

    it('should call addLogToTerminal every 3 seconds', fakeAsync(() => {
        const addLogSpy = spyOn(component, 'addLogToTerminal');

        component.startLogSimulation();
        tick(3000);

        expect(addLogSpy).toHaveBeenCalled();

        discardPeriodicTasks();
    }));

    it('should add a log to the terminal', () => {
        const terminalElement = fixture.nativeElement.querySelector('div');

        const testLog = 'Test log entry';
        component.addLogToTerminal(terminalElement, testLog);

        const logElements = terminalElement.getElementsByTagName('p');
        expect(logElements.length).toBe(1);
        expect(logElements[0].textContent).toBe(testLog);
    });
});
