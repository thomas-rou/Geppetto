import { Component } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { NotificationPromptComponent } from '../notification-prompt/notification-prompt.component';

@Component({
    selector: 'app-root',
    standalone: true,
    templateUrl: './app.component.html',
    styleUrls: ['./app.component.scss'],
    imports: [RouterOutlet, NotificationPromptComponent]
})
export class AppComponent {}
