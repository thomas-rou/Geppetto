import { Injectable } from '@angular/core';

@Injectable({
    providedIn: 'root',
})
export class ThemeService {
    private isDarkMode: boolean = true;

    toggleTheme() {
        this.isDarkMode = !this.isDarkMode;
        const theme = this.getCurrentTheme();
        document.documentElement.setAttribute('data-theme', theme);
    }

    getCurrentTheme() {
        return this.isDarkMode ? 'dark-theme' : 'light-theme';
    }
}
