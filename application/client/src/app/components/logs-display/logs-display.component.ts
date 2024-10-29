import { Component, AfterViewInit, ViewChild, ElementRef } from '@angular/core';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { collapseExpandAnimation } from 'src/assets/CollapseExpand';

@Component({
    selector: 'app-logs-display',
    standalone: true,
    imports: [],
    templateUrl: './logs-display.component.html',
    styleUrl: './logs-display.component.scss',
    animations: [collapseExpandAnimation]
})
export class LogsDisplayComponent implements AfterViewInit {
    @ViewChild('logTerminal') logTerminal!: ElementRef;
    isCollapsed = true;

    constructor(private robotCommunicationService: RobotCommunicationService) {}

    ngAfterViewInit() {
        this.robotCommunicationService.onLog().subscribe(log => {
            this.addLogToTerminal(this.logTerminal.nativeElement, log);
        });
    }

    toggleCollapse() {
        this.isCollapsed = !this.isCollapsed;
    }

    addLogToTerminal(terminal: HTMLElement, log: string) {
        const logElement = document.createElement('p');
        logElement.textContent = log;
        terminal.appendChild(logElement);
        terminal.scrollTop = terminal.scrollHeight;
    }
}
