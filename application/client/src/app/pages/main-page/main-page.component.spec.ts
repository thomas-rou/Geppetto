import { ComponentFixture, TestBed } from '@angular/core/testing';
import { HttpClientModule } from '@angular/common/http';
import { MainPageComponent } from './main-page.component';
import { ThemeService } from '@app/services/theme/theme.service';
import { ControlPanelComponent } from '@app/components/control-panel/control-panel.component';
import { StatusDisplayComponent } from '@app/components/status-display/status-display.component';
import { MapDisplayComponent } from '@app/components/map-display/map-display.component';
import { LogsDisplayComponent } from '@app/components/logs-display/logs-display.component';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';

describe('MainPageComponent', () => {
    let component: MainPageComponent;
    let fixture: ComponentFixture<MainPageComponent>;
    let themeService: jasmine.SpyObj<ThemeService>;

    beforeEach(async () => {
        const themeServiceSpy = jasmine.createSpyObj('ThemeService', ['toggleTheme']);

        await TestBed.configureTestingModule({
            imports: [HttpClientModule, BrowserAnimationsModule, MainPageComponent, ControlPanelComponent, StatusDisplayComponent, MapDisplayComponent, LogsDisplayComponent],
            providers: [{ provide: ThemeService, useValue: themeServiceSpy }],
        }).compileComponents();

        fixture = TestBed.createComponent(MainPageComponent);
        component = fixture.componentInstance;
        themeService = TestBed.inject(ThemeService) as jasmine.SpyObj<ThemeService>;
        fixture.detectChanges();
    });

    it('should create', () => {
        expect(component).toBeTruthy();
    });

    it('should call toggleTheme on ThemeService when toggleTheme is called', () => {
        component.toggleTheme();
        expect(themeService.toggleTheme).toHaveBeenCalled();
    });
});
