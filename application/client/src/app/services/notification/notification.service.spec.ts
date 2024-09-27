import { TestBed } from '@angular/core/testing';
import { NotificationService } from './notification.service';
import { take } from 'rxjs/operators';
import { Observable } from 'rxjs';

describe('NotificationService', () => {
  let service: NotificationService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(NotificationService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('should send notification', (done) => {
    const testMessage = 'Test notification';
    service.getNotifications().pipe(take(1)).subscribe((message) => {
      expect(message).toBe(testMessage);
      done();
    });
    service.sendNotification(testMessage);
  });

  it('should return notifications as observable', () => {
    const notifications$ = service.getNotifications();
    expect(notifications$).toBeTruthy();
    expect(notifications$).toBeInstanceOf(Observable);
  });
});
