import { ComponentFixture, TestBed } from '@angular/core/testing';
import { ElementRef } from '@angular/core';
import { LogsPageComponent } from './logs-page.component';
import { MissionService } from '@app/services/mission/mission.service';
import { ThemeService } from '@app/services/theme/theme.service';

describe('LogsPageComponent', () => {
  let component: LogsPageComponent;
  let fixture: ComponentFixture<LogsPageComponent>;
  let missionServiceMock: any;
  let themeServiceMock: any;

  beforeEach(async () => {
    missionServiceMock = {
      getMissionLogs: jasmine.createSpy('getMissionLogs'),
      handleMissionLogs: jasmine.createSpy('handleMissionLogs').and.returnValue(Promise.resolve()),
      missions: [
        {
          id: 1,
          logs: [
            { source: 'source1', log_type: 'type1', message: 'message1' },
            { source: 'source2', log_type: 'type2', message: 'message2' }
          ]
        }
      ]
    };

    themeServiceMock = {
      toggleTheme: jasmine.createSpy('toggleTheme')
    };

    await TestBed.configureTestingModule({
      providers: [
        { provide: MissionService, useValue: missionServiceMock },
        { provide: ThemeService, useValue: themeServiceMock }
      ]
    }).compileComponents();

    fixture = TestBed.createComponent(LogsPageComponent);
    component = fixture.componentInstance;
    component.logTerminal = new ElementRef(document.createElement('div'));
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should call getMissionLogs and loadMissionLogs on ngOnInit', () => {
    spyOn(component, 'loadMissionLogs');
    component.ngOnInit();
    expect(missionServiceMock.getMissionLogs).toHaveBeenCalled();
    expect(component.loadMissionLogs).toHaveBeenCalled();
  });

  it('should load mission logs and add them to the terminal', async () => {
    spyOn(component, 'addLogToTerminal');
    await component.loadMissionLogs();
    expect(missionServiceMock.handleMissionLogs).toHaveBeenCalled();
    expect(component.addLogToTerminal).toHaveBeenCalledWith('Mission ID: 1', true);
    expect(component.addLogToTerminal).toHaveBeenCalledWith('  source1 - type1: message1', false);
    expect(component.addLogToTerminal).toHaveBeenCalledWith('  source2 - type2: message2', false);
  });

  it('should toggle theme', () => {
    component.toggleTheme();
    expect(themeServiceMock.toggleTheme).toHaveBeenCalled();
  });
});