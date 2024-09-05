import { Component, AfterViewInit, Renderer2, ElementRef } from '@angular/core';
import { ThemeService } from '@app/services/theme.service';

@Component({
  selector: 'app-main-page',
  standalone: true,
  imports: [],
  templateUrl: './main-page.component.html',
  styleUrls: ['./main-page.component.scss']
})
export class MainPageComponent implements AfterViewInit {

  constructor(private renderer: Renderer2, private el: ElementRef, private themeService: ThemeService) {}

  ngAfterViewInit() {
    const startMissionBtn = this.el.nativeElement.querySelector('#start-mission');
    const stopMissionBtn = this.el.nativeElement.querySelector('#stop-mission');
    const returnHomeBtn = this.el.nativeElement.querySelector('#return-home');
    const updateSoftwareBtn = this.el.nativeElement.querySelector('#update-software');

    this.renderer.listen(startMissionBtn, 'click', () => {
      alert('Mission started!');
    });

    this.renderer.listen(stopMissionBtn, 'click', () => {
      alert('Mission stopped!');
    });

    this.renderer.listen(returnHomeBtn, 'click', () => {
      alert('Returning home!');
    });

    this.renderer.listen(updateSoftwareBtn, 'click', () => {
      alert('Software updated!');
    });
  }
  toggleTheme() {
    this.themeService.toggleTheme();
  }
}
