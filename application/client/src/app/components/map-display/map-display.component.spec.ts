import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MapDisplayComponent } from './map-display.component';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';

describe('MapDisplayComponent', () => {
    let component: MapDisplayComponent;
    let fixture: ComponentFixture<MapDisplayComponent>;

    beforeEach(async () => {
        await TestBed.configureTestingModule({
            imports: [MapDisplayComponent, BrowserAnimationsModule],
        }).compileComponents();

        fixture = TestBed.createComponent(MapDisplayComponent);
        component = fixture.componentInstance;
        fixture.detectChanges();
    });

    it('should create', () => {
        expect(component).toBeTruthy();
    });

    it('should toggle collapse state', () => {
        component.isCollapsed = false;
        component.toggleCollapse();
        expect(component.isCollapsed).toBeTrue();
    });
});
