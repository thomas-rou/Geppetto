import { TestBed } from '@angular/core/testing';

import { RobotCommunicationService } from './robot-communication.service';

describe('RobotCommunicationService', () => {
  let service: RobotCommunicationService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(RobotCommunicationService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });
});
