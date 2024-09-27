import { ComponentFixture, TestBed } from '@angular/core/testing';

import { StartMissionPopupComponent } from './start-mission-popup.component';

describe('StartMissionPopupComponent', () => {
  let component: StartMissionPopupComponent;
  let fixture: ComponentFixture<StartMissionPopupComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [StartMissionPopupComponent]
    })
    .compileComponents();

    fixture = TestBed.createComponent(StartMissionPopupComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
