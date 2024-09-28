import { ComponentFixture, TestBed } from '@angular/core/testing';
import { NotificationPromptComponent } from './notification-prompt.component';
import { NotificationService } from '@app/services/notification/notification.service';
import { of } from 'rxjs';
import { CommonModule } from '@angular/common';

describe('NotificationPromptComponent', () => {
    let component: NotificationPromptComponent;
    let fixture: ComponentFixture<NotificationPromptComponent>;
    let notificationService: jasmine.SpyObj<NotificationService>;

    beforeEach(async () => {
        const notificationServiceSpy = jasmine.createSpyObj('NotificationService', ['getNotifications']);

        await TestBed.configureTestingModule({
            imports: [CommonModule],
            providers: [{ provide: NotificationService, useValue: notificationServiceSpy }],
        }).compileComponents();

        fixture = TestBed.createComponent(NotificationPromptComponent);
        component = fixture.componentInstance;
        notificationService = TestBed.inject(NotificationService) as jasmine.SpyObj<NotificationService>;
    });

    it('should create', () => {
        expect(component).toBeTruthy();
    });

    it('should add a notification on init', () => {
        const message = 'Test notification';
        notificationService.getNotifications.and.returnValue(of(message));
        spyOn(component, 'addNotification');

        component.ngOnInit();

        expect(component.addNotification).toHaveBeenCalledWith(message);
    });

    it('should remove a notification', () => {
        const message = 'Test notification';
        const timeoutId = setTimeout(() => {}, 3000);
        component.notifications.push({ message, timeoutId });

        component.removeNotification({ message, timeoutId });

        expect(component.notifications.length).toBe(0);
    });

    it('should add and remove a notification after timeout', (done) => {
        const message = 'Test notification';

        component.addNotification(message);

        expect(component.notifications.length).toBe(1);
        expect(component.notifications[0].message).toBe(message);

        setTimeout(() => {
            expect(component.notifications.length).toBe(0);
            done();
        }, 3100);
    });
});
