import { TestBed } from '@angular/core/testing';
import { StatusDisplayComponent } from './status-display.component';
import { RobotManagementService } from '@app/services/robot-management/robot-management.service';
import { RobotCommunicationService } from '@app/services/robot-communication/robot-communication.service';
import { Robot } from '@app/classes/robot';
import { RobotStatus } from '@app/enums/robot-status';

describe('StatusDisplayComponent', () => {
    let component: StatusDisplayComponent;
    let robotCommunicationService: jasmine.SpyObj<RobotCommunicationService>;
    let robotManagementService: jasmine.SpyObj<RobotManagementService>;

    beforeEach(() => {
        const robotCommunicationSpy = jasmine.createSpyObj('RobotCommunicationService', ['identifyRobot']);
        const robotManagementSpy = jasmine.createSpyObj('RobotManagementService', [], {
            robot1: { id: '1', name: 'Robot 1', status: RobotStatus.Idle, battery: 100, position: { x: 0, y: 0 }, orientation: 0.0 } as Robot,
            robot2: { id: '2', name: 'Robot 2', status: RobotStatus.Idle, battery: 100, position: { x: 0, y: 0 }, orientation: 0.0 } as Robot,
        });

        TestBed.configureTestingModule({
            providers: [
                { provide: RobotCommunicationService, useValue: robotCommunicationSpy },
                { provide: RobotManagementService, useValue: robotManagementSpy },
            ],
        }).compileComponents();

        const fixture = TestBed.createComponent(StatusDisplayComponent);
        component = fixture.componentInstance;
        robotCommunicationService = TestBed.inject(RobotCommunicationService) as jasmine.SpyObj<RobotCommunicationService>;
        robotManagementService = TestBed.inject(RobotManagementService) as jasmine.SpyObj<RobotManagementService>;
    });

    it('should create', () => {
        expect(component).toBeTruthy();
    });

    it('should call identifyRobot on RobotCommunicationService with correct robot id', () => {
        const robot: Robot = new Robot('1', 'Robot 1', RobotStatus.Idle, 100, { x: 0, y: 0 }, 0.0);
        component.identifyRobot(robot);
        expect(robotCommunicationService.identifyRobot).toHaveBeenCalledWith(robot.id);
    });

    it('should return robot1 from RobotManagementService', () => {
        expect(component.robot1).toEqual(robotManagementService.robot1);
    });

    it('should return robot2 from RobotManagementService', () => {
        expect(component.robot2).toEqual(robotManagementService.robot2);
    });
});
