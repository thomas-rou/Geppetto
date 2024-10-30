import { ComponentFixture, TestBed } from '@angular/core/testing';

import { LogsPageComponent } from './logs-page.component';

describe('LogsPageComponent', () => {
  let component: LogsPageComponent;
  let fixture: ComponentFixture<LogsPageComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [LogsPageComponent]
    })
    .compileComponents();

    fixture = TestBed.createComponent(LogsPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
