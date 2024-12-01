import { TestBed } from '@angular/core/testing';
import { ThemeService } from './theme.service';

describe('ThemeService', () => {
    let service: ThemeService;

    beforeEach(() => {
        TestBed.configureTestingModule({});
        service = TestBed.inject(ThemeService);
    });

    it('should be created', () => {
        expect(service).toBeTruthy();
    });

    it('should toggle theme from dark to light', () => {
        service.toggleTheme();
        expect(service.getCurrentTheme()).toBe('light');
        expect(document.documentElement.getAttribute('data')).toBe('light');
    });

    it('should toggle theme from light to dark', () => {
        service.toggleTheme();
        service.toggleTheme();
        expect(service.getCurrentTheme()).toBe('dark');
        expect(document.documentElement.getAttribute('data')).toBe('dark');
    });

    it('should return the current theme as dark-theme initially', () => {
        expect(service.getCurrentTheme()).toBe('dark');
    });

    it('should return the current theme as light-theme after toggling', () => {
        service.toggleTheme();
        expect(service.getCurrentTheme()).toBe('light');
    });
});
