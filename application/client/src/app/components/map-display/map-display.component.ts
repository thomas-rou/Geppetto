import { Component, ElementRef, ViewChild, OnInit } from '@angular/core';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { collapseExpandAnimation } from 'src/assets/CollapseExpand';
import { RobotPose } from '@common/interfaces/RobotPose';

interface MapMetaData {
    width: number;
    height: number;
    resolution: number;
    origin: { x: number; y: number; z: number };
}

interface OccupancyGrid {
    header: {
        stamp: { sec: number; nsec: number };
        frame_id: string;
    };
    info: MapMetaData;
    data: Int8Array;
}

@Component({
    selector: 'app-map-display',
    standalone: true,
    templateUrl: './map-display.component.html',
    styleUrls: ['./map-display.component.scss'],
    animations: [collapseExpandAnimation]
})
export class MapDisplayComponent implements OnInit {
    @ViewChild('mapCanvas', { static: true }) mapCanvas!: ElementRef<HTMLCanvasElement>;
    isCollapsed = true;
    robotPose: RobotPose[] = [];

    constructor(private robotCommunicationService: RobotCommunicationService) {}

    ngOnInit(): void {
        this.robotCommunicationService.onLiveMap().subscribe((occupancyGrid: OccupancyGrid) => {
            this.drawMap(occupancyGrid);
        });

        this.robotCommunicationService.onRobotPositions().subscribe((robotPose: RobotPose[]) => {
            this.robotPose = [robotPose[0], robotPose[1]];
            this.drawRobotPositions();
        });
    }

    toggleCollapse() {
        this.isCollapsed = !this.isCollapsed;
    }

    drawMap(occupancyGrid: OccupancyGrid): void {
        const canvas = this.mapCanvas.nativeElement;
        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        const { width, height } = occupancyGrid.info;
        canvas.width = width;
        canvas.height = height;

        for (let y = 0; y < height; y++) {
            for (let x = 0; x < width; x++) {
                const index = y * width + x;
                const cellValue = occupancyGrid.data[index];
                if (cellValue === -1) {
                    ctx.fillStyle = 'gray';
                }
                else if (cellValue === 0) {
                    ctx.fillStyle = 'white';
                }
                else {
                    ctx.fillStyle = 'black';
                }
                ctx.fillRect(x, y, 1, 1);
            }
        }

        this.drawRobotPositions();
    }

    drawRobotPositions(): void {
        const canvas = this.mapCanvas.nativeElement;
        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        this.robotPose.forEach(robot => {
            ctx.fillStyle = 'red';
            ctx.beginPath();
            ctx.arc(robot.position.x, robot.position.y, 5, 0, 2 * Math.PI);
            ctx.fill();
        });
    }
}
