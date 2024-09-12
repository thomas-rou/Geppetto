import { Component, AfterViewInit } from '@angular/core';

@Component({
  selector: 'app-logs-display',
  standalone: true,
  imports: [],
  templateUrl: './logs-display.component.html',
  styleUrl: './logs-display.component.scss'
})
export class LogsDisplayComponent implements AfterViewInit {
  ngAfterViewInit() {
    this.startLogSimulation();
  }

  startLogSimulation() {
    const terminal = document.getElementById('log-terminal');
    let logCount = 0;

    setInterval(() => {
      const log = `$ Log entry #${++logCount}: Robot 1 is active.`;
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
}