import { Component } from '@angular/core';
import { ThemeService } from '@app/services/theme.service';
import { ControlPanelComponent } from '@app/components/control-panel/control-panel.component';
import { StatusDisplayComponent } from "@app/components/status-display/status-display.component";
import { MapDisplayComponent } from "../../components/map-display/map-display.component";
import { LogsDisplayComponent } from "../../components/logs-display/logs-display.component";

@Component({
    selector: 'app-main-page',
    standalone: true,
    imports: [ControlPanelComponent, StatusDisplayComponent, MapDisplayComponent, LogsDisplayComponent],
    templateUrl: './main-page.component.html',
    styleUrls: ['./main-page.component.scss'],
})
export class MainPageComponent {


    constructor(
        private themeService: ThemeService
    ){}

    // ngAfterViewInit() {
    //     this.startLogSimulation();
    // }
    
    // startLogSimulation() {
    //     const terminal = document.getElementById('log-terminal');
    //     let logCount = 0;
        
    //     setInterval(() => {
    //         const log = `$ Log entry #${++logCount}: Robot 1 is ${this.robot1.status}. Battery: ${this.robot1.battery}%.`;
    //         this.addLogToTerminal(terminal, log);
    //     }, 3000);
    // }
    
    // addLogToTerminal(terminal: HTMLElement | null, log: string) {
    //     if (terminal) {
    //         const logElement = document.createElement('p');
    //         logElement.textContent = log;
    //         terminal.appendChild(logElement);
    //         terminal.scrollTop = terminal.scrollHeight;
    //     }
    // }

    toggleTheme() {
        this.themeService.toggleTheme();
    }
}