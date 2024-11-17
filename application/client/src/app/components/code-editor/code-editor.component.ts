import { Component, OnInit, HostListener } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { collapseExpandAnimation } from 'src/assets/CollapseExpand';
import { CodeEditor, Theme } from '@acrodata/code-editor';
import { SocketHandlerService } from '@app/services/socket-handler/socket-handler.service';
import { MissionService } from '@app/services/mission/mission.service';
import { languages } from '@codemirror/language-data';
import { ThemeService } from '@app/services/theme/theme.service';

@Component({
  selector: 'app-code-editor',
  template: `<code-editor [(ngModel)]="value" />`,
  standalone: true,
  imports: [
    CommonModule,
    FormsModule,
    CodeEditor
  ],
  templateUrl: './code-editor.component.html',
  styleUrls: ['./code-editor.component.scss'],
  animations: [collapseExpandAnimation],
})
export class CodeEditorComponent implements OnInit {
  isCollapsed : boolean = true;
  value : string = '';
  theme: Theme = 'dark';

  languages = languages;

  constructor(
    private socketService: SocketHandlerService,
    private missionService: MissionService,
    private themeService: ThemeService
  ) {}

  ngOnInit() {
    this.socketService.connect();
    this.socketService.on('codeFileContent', (data: string) => {
      this.value = data;
      this.missionService.setInitialCode(this.value);
    });
    this.socketService.on('codeFileError', (error: string) => {
      console.error('Error loading code file:', error);
    });
    this.loadCodeFile();

    this.theme = this.themeService.getCurrentTheme();
    this.themeService.themeChanged.subscribe((newTheme: Theme) => {
      this.theme = newTheme;
    });
  }

  loadCodeFile() {
    this.socketService.send('getCodeFile', this.missionService.filenameToModify);
  }

  toggleCollapse() {
    this.isCollapsed = !this.isCollapsed;
  }

  onEditorChange(newValue: string) {
    this.value = newValue;
    this.missionService.setNewCode(this.value);
  }

  @HostListener('document:keydown.control.f', ['$event'])
  handleCtrlF(event: KeyboardEvent) {
    event.preventDefault();
    const closeButton = document.querySelector('button[name="close"][aria-label="close"]');
    if (closeButton) {
      (closeButton as HTMLElement).click();
    }
  }
}
