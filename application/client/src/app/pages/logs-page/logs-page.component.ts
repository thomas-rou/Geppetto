import { Component, ElementRef, OnInit, ViewChild } from '@angular/core';
import { MissionService } from '@app/services/mission/mission.service';
import { ThemeService } from '@app/services/theme/theme.service';
import { Mission } from '@common/interfaces/Mission';
import { LogsDisplayComponent } from "@app/components/logs-display/logs-display.component";
import { MapDisplayComponent } from "../../components/map-display/map-display.component";


@Component({
    selector: 'app-logs-page',
    standalone: true,
    imports: [LogsDisplayComponent, MapDisplayComponent],
    templateUrl: './logs-page.component.html',
    styleUrl: './logs-page.component.scss',
})
export class LogsPageComponent implements OnInit {
    @ViewChild('logTerminal') logTerminal!: ElementRef;
    missions: Mission[] = [];
    mission: Mission;

    constructor(
        private themeService: ThemeService,
        public missionService: MissionService,
    ) {}

    ngOnInit() {
        this.missionService.getMissionLogs();
        this.loadMissionLogs();
    }

    async loadMissionLogs() {
        await this.missionService.handleMissionLogs();
        this.missionService.missions.forEach((mission) => {
            // this.addLogToTerminal(`Mission ID: ${mission.id}`, true);
            // mission.logs.forEach((log) => {
            //     this.addLogToTerminal(`  ${log.source} - ${log.log_type}: ${log.message}`, false);
            // });
        });
    }

    addLogToTerminal(log: string, isMainLine: boolean) {
        const logElement = document.createElement('p');
        logElement.textContent = log;
        if (isMainLine) {
            logElement.style.fontWeight = 'bold';
        } else {
            logElement.style.marginLeft = '20px';
        }
        this.logTerminal.nativeElement.appendChild(logElement);
        this.logTerminal.nativeElement.scrollTop = this.logTerminal.nativeElement.scrollHeight;
    }

    toggleTheme() {
        this.themeService.toggleTheme();
    }
}
