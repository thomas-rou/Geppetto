import { Component, EventEmitter, Output } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { GeofenceCoord } from '@common/types/GeofenceCoord';

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

  coords: GeofenceCoord;

  onGeofence() {
    this.coords = {
      X1: this.X1,
      Y1: this.X1,
      X2: this.X1,
      Y2: this.X1,
    }
    this.geofence.emit(this.coords);
}
  
  onCancel() {
    this.cancelMission.emit();
  }
}
