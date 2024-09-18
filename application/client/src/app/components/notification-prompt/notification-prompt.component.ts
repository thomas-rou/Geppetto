//timing logic implemented using tool we found online: https://chatgpt.com/
import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { NotificationService } from '../../services/notification.service';

@Component({
  selector: 'app-notification-prompt',
  templateUrl: './notification-prompt.component.html',
  styleUrls: ['./notification-prompt.component.css'],
  standalone: true,
  imports: [CommonModule]
})
export class NotificationPromptComponent implements OnInit {
  notifications: { message: string; timeoutId: any }[] = [];

  constructor(private notificationService: NotificationService) {}

  ngOnInit(): void {
    this.notificationService.getNotifications().subscribe((message: string) => {
      this.addNotification(message);
    });
  }

  removeNotification(notification: { message: string; timeoutId: any }): void {
    clearTimeout(notification.timeoutId); 
    this.notifications = this.notifications.filter(n => n.timeoutId !== notification.timeoutId);
  }

  addNotification(message: string): void {
    const timeoutId = setTimeout(() => {
      this.removeNotification({ message, timeoutId });
    }, 3000);

    this.notifications.push({ message, timeoutId });
  }
}
