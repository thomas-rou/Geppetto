import { Component, ElementRef, ViewChild, OnInit } from '@angular/core';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { collapseExpandAnimation } from 'src/assets/CollapseExpand';
import { RobotPose } from '@common/interfaces/RobotPose';
import { OccupancyGrid, MapMetaData } from '@common/interfaces/LiveMap';

const ROBOT_RADIUS = 5;
const ROBOT_START_ANGLE = 0;
const ROBOT_END_ANGLE = 2 * Math.PI;
const HEX_LETTERS = '0123456789ABCDEF';
const COLOR_LENGTH = 6;
const HEX_BASE = 16;

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
    private robotPoses: { [topic: string]: RobotPose[] } = {};
    private topicColors: { [key: string]: string } = {};
    private occupancyGridInfo: MapMetaData;

    constructor(private robotCommunicationService: RobotCommunicationService) {}

    ngOnInit(): void {
        this.subscribeToLiveMap();
        this.subscribeToRobotPositions();
    }

    toggleCollapse() {
        this.isCollapsed = !this.isCollapsed;
    }

    private subscribeToLiveMap(): void {
        this.robotCommunicationService.onLiveMap().subscribe((occupancyGrid: OccupancyGrid) => {
            this.occupancyGridInfo = occupancyGrid.info;
            this.drawMap(occupancyGrid);
        });
    }

    private subscribeToRobotPositions(): void {
        this.robotCommunicationService.onRobotPositions().subscribe((robotPose: RobotPose) => {
            if (robotPose.topic) {
                this.robotPoses[robotPose.topic] = [robotPose];
            }
            this.drawRobotPositions();
        });
    }

    private drawMap(occupancyGrid: OccupancyGrid): void {
        const canvas = this.mapCanvas.nativeElement;
        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        this.setCanvasDimensions(canvas);
        this.drawOccupancyGrid(ctx, occupancyGrid);
        this.drawRobotPositions();
    }

    private setCanvasDimensions(canvas: HTMLCanvasElement): void {
        const { width, height } = this.occupancyGridInfo;
        canvas.width = width;
        canvas.height = height;
    }

    private drawOccupancyGrid(ctx: CanvasRenderingContext2D, occupancyGrid: OccupancyGrid): void {
        const { width, height } = this.occupancyGridInfo;

        for (let y = 0; y < height; y++) {
            for (let x = 0; x < width; x++) {
                const index = y * width + x;
                const cellValue = occupancyGrid.data[index];
                ctx.fillStyle = this.getCellColor(cellValue);
                ctx.fillRect(x, y, 1, 1);
            }
        }
    }

    private getCellColor(cellValue: number): string {
        switch (cellValue) {
            case -1: return 'gray';
            case 0: return 'white';
            default: return 'black';
        }
    }

    private drawRobotPositions(): void {
        const canvas = this.mapCanvas.nativeElement;
        const ctx = canvas.getContext('2d');
        if (!ctx || !this.occupancyGridInfo) return;

        const { origin, resolution } = this.occupancyGridInfo;

        Object.keys(this.robotPoses).forEach(topic => {
            const color = this.getColorForTopic(topic);
            this.robotPoses[topic].forEach(robot => {
                if (robot && robot.position) {
                    this.drawRobot(ctx, robot, color, origin, resolution, canvas.height);
                }
            });
        });
    }

    private drawRobot(ctx: CanvasRenderingContext2D, robot: RobotPose, color: string, origin: any, resolution: number, canvasHeight: number): void {
        const x = (robot.position.x - origin.position.x) / resolution;
        const y = canvasHeight - (robot.position.y - origin.position.y) / resolution;

        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(x, y, ROBOT_RADIUS, ROBOT_START_ANGLE, ROBOT_END_ANGLE);
        ctx.fill();
    }

    getColorForTopic(topic: string): string {
        if (!this.topicColors[topic]) {
            this.topicColors[topic] = this.getRandomColor();
        }
        return this.topicColors[topic];
    }

    getRandomColor(): string {
        let color = '#';
        for (let i = 0; i < COLOR_LENGTH; i++) {
            color += HEX_LETTERS[Math.floor(Math.random() * HEX_BASE)];
        }
        return color;
    }
}
