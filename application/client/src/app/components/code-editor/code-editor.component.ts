import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { collapseExpandAnimation } from 'src/assets/CollapseExpand';
import { CodeEditor } from '@acrodata/code-editor';
import { SocketHandlerService } from '@app/services/socket-handler/socket-handler.service';

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
  isCollapsed = true;
  value = '';

  constructor(private socketService: SocketHandlerService) {}

  ngOnInit() {
    this.socketService.connect();

    this.socketService.on<string>('codeFileContent', (data: string) => {
      this.value = data;
    });

    this.socketService.on<string>('codeFileError', (error: string) => {
      console.error('Error loading code file:', error);
    });

    this.loadCodeFile();
  }

  loadCodeFile() {
    this.socketService.send('getCodeFile');
  }

  toggleCollapse() {
    this.isCollapsed = !this.isCollapsed;
  }
}
