import { Component, AfterViewInit, ViewChild, ElementRef } from '@angular/core';

@Component({
    selector: 'app-logs-display',
    standalone: true,
    imports: [],
    templateUrl: './logs-display.component.html',
    styleUrl: './logs-display.component.scss',
})
export class LogsDisplayComponent implements AfterViewInit {
    @ViewChild('logTerminal') logTerminal!: ElementRef;

    ngAfterViewInit() {
        this.startLogSimulation();
    }

    startLogSimulation() {
        let logCount = 0;

        setInterval(() => {
            const log = `$ Log entry #${++logCount}: Robot 1 is active.`;
            this.addLogToTerminal(this.logTerminal.nativeElement, log);
        }, 3000);
    }

    addLogToTerminal(terminal: HTMLElement, log: string) {
        const logElement = document.createElement('p');
        logElement.textContent = log;
        terminal.appendChild(logElement);
        terminal.scrollTop = terminal.scrollHeight;
    }
}
