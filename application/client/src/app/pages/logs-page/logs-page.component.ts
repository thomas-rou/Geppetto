import { Component, ElementRef, OnInit, ViewChild } from '@angular/core';
import { MissionService } from '@app/services/mission/mission.service';
import { ThemeService } from '@app/services/theme/theme.service';
import { LogMessage } from '@common/interfaces/LogMessage';

@Component({
  selector: 'app-logs-page',
  standalone: true,
  imports: [],
  templateUrl: './logs-page.component.html',
  styleUrl: './logs-page.component.scss'
})
export class LogsPageComponent implements OnInit {
  @ViewChild('logTerminal') logTerminal!: ElementRef;
  logs: LogMessage[] = [];

  constructor(
    private themeService: ThemeService,
    private missionService: MissionService,
  ) {}

  ngOnInit() {
    this.missionService.getMissionLogs();
    this.missionService.missionLogs.forEach(log => this.addLogToTerminal(`${log.date} - ${log.source} - ${log.log_type}: ${log.message}`));
  }

  addLogToTerminal(log: string) {
    const logElement = document.createElement('p');
    logElement.textContent = log;
    this.logTerminal.nativeElement.appendChild(logElement);
    this.logTerminal.nativeElement.scrollTop = this.logTerminal.nativeElement.scrollHeight;
  }

  toggleTheme() {
    this.themeService.toggleTheme();
  }
}
