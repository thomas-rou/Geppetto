import { TestBed } from '@angular/core/testing';

import { GeofenceService } from './geofence.service';

describe('GeofenceService', () => {
  let service: GeofenceService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(GeofenceService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });
});
