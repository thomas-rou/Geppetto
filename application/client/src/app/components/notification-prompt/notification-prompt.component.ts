import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { NotificationService } from '@app/services/notification/notification.service';

@Component({
    selector: 'app-notification-prompt',
    templateUrl: './notification-prompt.component.html',
    styleUrls: ['./notification-prompt.component.scss'],
    standalone: true,
    imports: [CommonModule],
})
export class NotificationPromptComponent implements OnInit {
    notifications: { message: string; timeoutId: ReturnType<typeof setTimeout> }[] = [];

    constructor(private notificationService: NotificationService) {}

    ngOnInit(): void {
        this.notificationService.getNotifications().subscribe((message: string) => {
            this.addNotification(message);
        });
    }

    removeNotification(notification: { message: string; timeoutId: ReturnType<typeof setTimeout> }): void {
        clearTimeout(notification.timeoutId);
        this.notifications = this.notifications.filter((n) => n.timeoutId !== notification.timeoutId);
    }

    addNotification(message: string): void {
        const timeoutId = setTimeout(() => {
            this.removeNotification({ message, timeoutId });
        }, 3000);

        this.notifications.push({ message, timeoutId });
    }
}
