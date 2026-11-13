import { useArm } from '../store/useArm';
import { FsmDiagram } from './FsmDiagram';

export function RunPanel() {
  const { send, tel, cycles, lastEvent } = useArm();
  return (
    <section className="panel">
      <h2>Run</h2>
      <div className="row">
        {['HOME', 'RUN', 'STOP', 'RESET'].map(c => <button key={c} className={c === 'RUN' ? 'primary' : c === 'RESET' ? 'warn' : ''} onClick={() => send(c)}>{c}</button>)}
        <span className="muted small">cycles <b>{cycles}</b></span>
      </div>
      <FsmDiagram active={tel?.state} />
      <div className="muted small">last event: {lastEvent || '—'}</div>
    </section>
  );
}
