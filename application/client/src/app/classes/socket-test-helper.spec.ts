import { SocketTestHelper } from '@app/classes/socket-test-helper';

describe('SocketTestHelper', () => {
    let socketTestHelper: SocketTestHelper;

    beforeEach(() => {
        socketTestHelper = new SocketTestHelper();
    });

    it('should register a callback when "on" is called', () => {
        const callback = jasmine.createSpy('callback');

        socketTestHelper.on('testEvent', callback);

        expect(socketTestHelper['callbacks'].has('testEvent')).toBeTrue();
        expect(socketTestHelper['callbacks'].get('testEvent')?.length).toBe(1);
    });

    it('should call the registered callback when "peerSideEmit" is called', () => {
        const callback = jasmine.createSpy('callback');

        socketTestHelper.on('testEvent', callback);
        socketTestHelper.peerSideEmit('testEvent', 'some data');

        expect(callback).toHaveBeenCalledWith('some data');
    });

    it('should not call any callback if no callback is registered for the event', () => {
        const callback = jasmine.createSpy('callback');

        socketTestHelper.peerSideEmit('unregisteredEvent', 'some data');

        expect(callback).not.toHaveBeenCalled();
    });

    it('should allow multiple callbacks for the same event', () => {
        const callback1 = jasmine.createSpy('callback1');
        const callback2 = jasmine.createSpy('callback2');

        socketTestHelper.on('testEvent', callback1);
        socketTestHelper.on('testEvent', callback2);

        socketTestHelper.peerSideEmit('testEvent', 'some data');

        expect(callback1).toHaveBeenCalledWith('some data');
        expect(callback2).toHaveBeenCalledWith('some data');
    });
});
