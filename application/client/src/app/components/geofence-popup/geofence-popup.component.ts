import { Component, EventEmitter, Output } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { GeofenceCoord } from '@common/types/GeofenceCoord';
import { GeofenceService } from '@app/services/geofence/geofence.service';

@Component({
  selector: 'app-geofence-popup',
  standalone: true,
  imports: [CommonModule, FormsModule],
  templateUrl: './geofence-popup.component.html',
  styleUrl: './geofence-popup.component.scss'
})
export class GeofencePopupComponent {
  @Output() geofence = new EventEmitter<GeofenceCoord>();
  @Output() cancelMission = new EventEmitter<void>();
  
  X1: number = 0;
  Y1: number = 0;
  X2: number = 0;
  Y2: number = 0;

  constructor(private geofenceService: GeofenceService) {}

  onGeofence() {
    const coords: GeofenceCoord = {
      X1: this.X1,
      Y1: this.Y1,
      X2: this.X2,
      Y2: this.Y2,
    };
    this.geofenceService.setGeofence(coords);
    this.geofence.emit(coords);
  }
  
  onCancel() {
    this.cancelMission.emit();
  }
}