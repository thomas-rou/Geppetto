import { Component, AfterViewInit } from '@angular/core';
import { ThemeService } from '@app/services/theme.service';
import { Robot } from '@app/classes/robot';
import { RobotStatus } from '@app/enums/robot-status';

@Component({
    selector: 'app-main-page',
    standalone: true,
    imports: [],
    templateUrl: './main-page.component.html',
    styleUrls: ['./main-page.component.scss'],
})
export class MainPageComponent implements AfterViewInit {
    robot1: Robot;
    robot2: Robot;

    constructor(
        private themeService: ThemeService
    ){
        this.robot1 = new Robot('Robot 1', RobotStatus.Idle, 100, { x: 0, y: 0 });
        this.robot2 = new Robot('Robot 2', RobotStatus.Idle, 100, { x: 0, y: 0 });
    }

    ngAfterViewInit() {
        this.startLogSimulation();
    }
    
    startLogSimulation() {
        const terminal = document.getElementById('log-terminal');
        let logCount = 0;
        
        setInterval(() => {
            const log = `$ Log entry #${++logCount}: Robot 1 is ${this.robot1.status}. Battery: ${this.robot1.battery}%.`;
            this.addLogToTerminal(terminal, log);
        }, 3000);
    }
    
    addLogToTerminal(terminal: HTMLElement | null, log: string) {
        if (terminal) {
            const logElement = document.createElement('p');
            logElement.textContent = log;
            terminal.appendChild(logElement);
            terminal.scrollTop = terminal.scrollHeight;
        }
    }

    startMission() {
        alert('Mission started!');
        this.robot1.status = RobotStatus.Active;
    }

    stopMission() {
        alert('Mission stopped!');
        this.robot1.status = RobotStatus.Idle;
    }

    returnHome() {
        alert('Returning home!');
    };

    updateSoftware() {
        alert('Software updated!');
    };

    toggleTheme() {
        this.themeService.toggleTheme();
    }
}