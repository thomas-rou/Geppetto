import { Component } from '@angular/core';
import { collapseExpandAnimation } from 'src/assets/CollapseExpand';

@Component({
    selector: 'app-map-display',
    standalone: true,
    imports: [],
    templateUrl: './map-display.component.html',
    styleUrl: './map-display.component.scss',
    animations: [collapseExpandAnimation],
})
export class MapDisplayComponent {
    isCollapsed = false;

    toggleCollapse() {
        this.isCollapsed = !this.isCollapsed;
    }
}
