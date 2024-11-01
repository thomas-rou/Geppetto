import { TestBed } from '@angular/core/testing';
import { MissionService } from './mission.service';
import { SocketHandlerService } from '@app/services/socket-handler/socket-handler.service';
import { ClientCommand } from '@common/enums/ClientCommand';
import { MissionType } from '@app/enums/MissionType';

describe('MissionService', () => {
  let service: MissionService;
  let socketServiceSpy: jasmine.SpyObj<SocketHandlerService>;

  beforeEach(() => {
    const spy = jasmine.createSpyObj('SocketHandlerService', ['isSocketAlive', 'connect', 'on', 'send', 'disconnect']);

    TestBed.configureTestingModule({
      providers: [
        MissionService,
        { provide: SocketHandlerService, useValue: spy }
      ]
    });

    service = TestBed.inject(MissionService);
    socketServiceSpy = TestBed.inject(SocketHandlerService) as jasmine.SpyObj<SocketHandlerService>;
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });



  it('should send mission logs request', () => {
    service.getMissionLogs();
    expect(socketServiceSpy.send).toHaveBeenCalledWith(ClientCommand.MissionLogs);
  });

  it('should get mission type', () => {
    service.setMissionType(MissionType.Physical);
    expect(service.getMissionType()).toBe(MissionType.Physical);
  });

  it('should set mission type', () => {
    service.setMissionType(MissionType.Simulation);
    expect(service.getMissionType()).toBe(MissionType.Simulation);
  });
});