import { Robot } from './robot';
import { RobotStatus } from '@app/enums/robot-status';

describe('Robot', () => {
    it('should create an instance', () => {
        expect(new Robot('TestRobot', RobotStatus.Active, 100, { x: 0, y: 0 })).toBeTruthy();
    });
});
