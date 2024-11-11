import { Theme } from '@acrodata/code-editor';
import { Injectable } from '@angular/core';
import { BehaviorSubject } from 'rxjs';

@Injectable({
    providedIn: 'root',
})
export class ThemeService {
    private isDarkMode: boolean = true;
    private themeSubject = new BehaviorSubject<Theme>(this.getCurrentTheme());
    themeChanged = this.themeSubject.asObservable();

    toggleTheme() {
        this.isDarkMode = !this.isDarkMode;
        const theme = this.getCurrentTheme();
        document.documentElement.setAttribute('data', theme);
        this.themeSubject.next(theme);
    }

    getCurrentTheme() {
        return this.isDarkMode ? 'dark' : 'light';
    }
}
