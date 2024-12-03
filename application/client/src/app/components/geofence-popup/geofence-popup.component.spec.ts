import { ComponentFixture, TestBed } from '@angular/core/testing';

import { GeofencePopupComponent } from './geofence-popup.component';

describe('GeofencePopupComponent', () => {
  let component: GeofencePopupComponent;
  let fixture: ComponentFixture<GeofencePopupComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [GeofencePopupComponent]
    })
    .compileComponents();

    fixture = TestBed.createComponent(GeofencePopupComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
