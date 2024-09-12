import { Component } from '@angular/core';
import { ThemeService } from '@app/services/theme.service';
import { ControlPanelComponent } from '@app/components/control-panel/control-panel.component';
import { StatusDisplayComponent } from '@app/components/status-display/status-display.component';
import { MapDisplayComponent } from '@app/components/map-display/map-display.component';
import { LogsDisplayComponent } from '@app/components/logs-display/logs-display.component';

@Component({
    selector: 'app-main-page',
    standalone: true,
    imports: [ControlPanelComponent, StatusDisplayComponent, MapDisplayComponent, LogsDisplayComponent],
    templateUrl: './main-page.component.html',
    styleUrls: ['./main-page.component.scss'],
})
export class MainPageComponent {
    constructor(private themeService: ThemeService) {}

    toggleTheme() {
        this.themeService.toggleTheme();
    }
}
