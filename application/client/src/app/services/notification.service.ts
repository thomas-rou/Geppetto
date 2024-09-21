import { Injectable } from '@angular/core';
import { Subject, Observable } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class NotificationService {
  private notificationSubject = new Subject<string>();

  sendNotification(message: string): void {
    this.notificationSubject.next(message);
  }

  getNotifications(): Observable<string> {
    return this.notificationSubject.asObservable();
  }
}