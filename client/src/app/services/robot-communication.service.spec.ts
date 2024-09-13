import { TestBed } from '@angular/core/testing';
import { HttpClientTestingModule, HttpTestingController } from '@angular/common/http/testing';
import { RobotCommunicationService } from './robot-communication.service';

describe('RobotCommunicationService', () => {
    let service: RobotCommunicationService;
    let httpMock: HttpTestingController;

    beforeEach(() => {
        TestBed.configureTestingModule({
            imports: [HttpClientTestingModule],
            providers: [RobotCommunicationService],
        });
        service = TestBed.inject(RobotCommunicationService);
        httpMock = TestBed.inject(HttpTestingController);
    });

    afterEach(() => {
        httpMock.verify();
    });

    it('should be created', () => {
        expect(service).toBeTruthy();
    });

    it('should send a POST request to start a mission', () => {
        const orientation = 'north';
        const position = { x: 10, y: 20 };

        service.startMission(orientation, position).subscribe();

        const req = httpMock.expectOne('http://localhost:3000/api');
        expect(req.request.method).toBe('POST');
        expect(req.request.body).toEqual({
            command: 'start_mission',
            mission_details: { orientation, position },
            timestamp: jasmine.any(String),
        });
        req.flush({});
    });

    it('should send a POST request to end a mission', () => {
        service.endMission().subscribe();

        const req = httpMock.expectOne('http://localhost:3000/api');
        expect(req.request.method).toBe('POST');
        expect(req.request.body).toEqual({
            command: 'end_mission',
            timestamp: jasmine.any(String),
        });
        req.flush({});
    });

    it('should send a POST request to update robot status', () => {
        const name = 'Robot1';
        const status = 'active';
        const position = { x: 15, y: 25 };

        service.updateRobot(name, status, position).subscribe();

        const req = httpMock.expectOne('http://localhost:3000/api');
        expect(req.request.method).toBe('POST');
        expect(req.request.body).toEqual({
            command: 'update',
            name,
            status,
            position,
            timestamp: jasmine.any(String),
        });
        req.flush({});
    });

    it('should send a POST request to return the robot to base', () => {
        service.returnToBase().subscribe();

        const req = httpMock.expectOne('http://localhost:3000/api');
        expect(req.request.method).toBe('POST');
        expect(req.request.body).toEqual({
            command: 'return_to_base',
            timestamp: jasmine.any(String),
        });
        req.flush({});
    });

    it('should send a POST request to update controller code', () => {
        const newCode = 'function newCode() {}';

        service.updateControllerCode(newCode).subscribe();

        const req = httpMock.expectOne('http://localhost:3000/api');
        expect(req.request.method).toBe('POST');
        expect(req.request.body).toEqual({
            command: 'update_controller_code',
            code: newCode,
            timestamp: jasmine.any(String),
        });
        req.flush({});
    });

    it('should send a POST request to notify robots to communicate (P2P)', () => {
        service.notifyRobotsToCommunicate().subscribe();

        const req = httpMock.expectOne('http://localhost:3000/api');
        expect(req.request.method).toBe('POST');
        expect(req.request.body).toEqual({
            command: 'P2P',
            timestamp: jasmine.any(String),
        });
        req.flush({});
    });

    it('should send a POST request to find the furthest robot', () => {
        const relativePoint = { x: 5, y: 10 };

        service.findFurthestRobot(relativePoint).subscribe();

        const req = httpMock.expectOne('http://localhost:3000/api');
        expect(req.request.method).toBe('POST');
        expect(req.request.body).toEqual({
            command: 'find_furthest',
            relative_point: relativePoint,
            timestamp: jasmine.any(String),
        });
        req.flush({});
    });
});
