import { Component, ElementRef, ViewChild, OnInit } from '@angular/core';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { collapseExpandAnimation } from 'src/assets/CollapseExpand';
import { RobotPose } from '@common/interfaces/RobotPose';
import { OccupancyGrid, MapMetaData } from '@common/interfaces/LiveMap';

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
        this.robotCommunicationService.onLiveMap().subscribe((occupancyGrid: OccupancyGrid) => {
            this.occupancyGridInfo = occupancyGrid.info;
            this.drawMap(occupancyGrid);
        });

        this.robotCommunicationService.onRobotPositions().subscribe((robotPose: RobotPose) => {
            if (robotPose.topic) {
                this.robotPoses[robotPose.topic] = [robotPose];
            }
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

        const { width, height } = this.occupancyGridInfo;
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
        if (!this.occupancyGridInfo) return;

        const { origin, resolution } = this.occupancyGridInfo;

        Object.keys(this.robotPoses).forEach(topic => {
            const color = this.getColorForTopic(topic);
            this.robotPoses[topic].forEach(robot => {
                if (robot && robot.position) {
                    const x = (robot.position.x - origin.position.x) / resolution;
                    const y = canvas.height - (robot.position.y - origin.position.y) / resolution;

                    ctx.fillStyle = color;
                    ctx.beginPath();
                    ctx.arc(x, y, 5, 0, 2 * Math.PI);
                    ctx.fill();
                }
            });
        });
    }

    getColorForTopic(topic: string): string {
        if (!this.topicColors[topic]) {
            this.topicColors[topic] = this.getRandomColor();
        }
        return this.topicColors[topic];
    }

    getRandomColor(): string {
        const letters = '0123456789ABCDEF';
        let color = '#';
        for (let i = 0; i < 6; i++) {
            color += letters[Math.floor(Math.random() * 16)];
        }
        return color;
    }
}
