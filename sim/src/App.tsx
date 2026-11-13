import { TopBar } from './panels/TopBar';
import { MovePanel } from './panels/MovePanel';
import { TeachPanel } from './panels/TeachPanel';
import { RunPanel } from './panels/RunPanel';
import { TelemetryPanel } from './panels/TelemetryPanel';
import { ConsolePanel } from './panels/ConsolePanel';
import { VirtualControls } from './panels/VirtualControls';
import { Workcell } from './scene/Workcell';

export function App() {
  return (
    <div className="app">
      <TopBar />
      <main className="grid">
        <div className="col left"><MovePanel /><TeachPanel /></div>
        <div className="col center">
          <section className="panel scene"><Workcell /></section>
          <TelemetryPanel />
        </div>
        <div className="col right"><RunPanel /><VirtualControls /><ConsolePanel /></div>
      </main>
    </div>
  );
}
