import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MapDisplayComponent } from './map-display.component';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { of } from 'rxjs';
import { OccupancyGrid } from '@common/interfaces/LiveMap';

describe('MapDisplayComponent', () => {
    let component: MapDisplayComponent;
    let fixture: ComponentFixture<MapDisplayComponent>;
    let robotCommunicationService: jasmine.SpyObj<RobotCommunicationService>;

    beforeEach(async () => {
        const robotCommunicationServiceSpy = jasmine.createSpyObj('RobotCommunicationService', ['onLiveMap']);

        await TestBed.configureTestingModule({
            imports: [BrowserAnimationsModule],
            providers: [
                { provide: RobotCommunicationService, useValue: robotCommunicationServiceSpy }
            ]
        }).compileComponents();

        fixture = TestBed.createComponent(MapDisplayComponent);
        component = fixture.componentInstance;
        robotCommunicationService = TestBed.inject(RobotCommunicationService) as jasmine.SpyObj<RobotCommunicationService>;
        robotCommunicationService.onLiveMap.and.returnValue(of({
            header: {
                stamp: { sec: 0, nsec: 0 },
                frame_id: 'map'
            },
            info: {
                width: 10,
                height: 10,
                resolution: 0.05,
                origin: { x: 0, y: 0, z: 0 }
            },
            data: new Int8Array(100)
        }));
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

    it('should subscribe to onLiveMap on ngOnInit', () => {
        const mockOccupancyGrid: OccupancyGrid = {
            header: {
                stamp: { sec: 0, nsec: 0 },
                frame_id: 'map'
            },
            info: {
                width: 10,
                height: 10,
                resolution: 0.05,
                origin: { x: 0, y: 0, z: 0 }
            },
            data: new Int8Array(100)
        };
        robotCommunicationService.onLiveMap.and.returnValue(of(mockOccupancyGrid));
        spyOn(component, 'drawMap');

        component.ngOnInit();

        expect(robotCommunicationService.onLiveMap).toHaveBeenCalled();
        expect(component.drawMap).toHaveBeenCalledWith(mockOccupancyGrid);
    });

    it('should handle null canvas context in drawMap', () => {
        const mockOccupancyGrid: OccupancyGrid = {
            header: {
                stamp: { sec: 0, nsec: 0 },
                frame_id: 'map'
            },
            info: {
                width: 10,
                height: 10,
                resolution: 0.05,
                origin: { x: 0, y: 0, z: 0 }
            },
            data: new Int8Array(100)
        };
        const canvas = document.createElement('canvas');
        component.mapCanvas = { nativeElement: canvas };

        spyOn(canvas, 'getContext').and.returnValue(null);

        component.drawMap(mockOccupancyGrid);

        expect(canvas.getContext).toHaveBeenCalledWith('2d');
    });

    // it('should draw map on canvas', () => {
    //     const mockOccupancyGrid: OccupancyGrid = {
    //         header: {
    //             stamp: { sec: 0, nsec: 0 },
    //             frame_id: 'map'
    //         },
    //         info: {
    //             width: 10,
    //             height: 10,
    //             resolution: 0.05,
    //             origin: { x: 0, y: 0, z: 0 }
    //         },
    //         data: new Int8Array(100)
    //     };
    //     const canvas = document.createElement('canvas');
    //     const ctx = canvas.getContext('2d') as CanvasRenderingContext2D; // Explicitly type ctx
    //     component.mapCanvas = { nativeElement: canvas };
    
    //     spyOn(canvas, 'getContext').and.returnValue(ctx);
    //     spyOn(ctx, 'fillRect');
    
    //     component.drawMap(mockOccupancyGrid);
    
    //     expect(canvas.getContext).toHaveBeenCalledWith('2d');
    //     expect(ctx.fillRect).toHaveBeenCalledTimes(100);
    // });
});