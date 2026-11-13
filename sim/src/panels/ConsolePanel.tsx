// Raw serial log (out / in / info, colour-coded) + free command input with history (↑ / ↓).
import { useEffect, useRef, useState } from 'react';
import { useArm } from '../store/useArm';

export function ConsolePanel() {
  const log = useArm(s => s.log);
  const send = useArm(s => s.send);
  const [cmd, setCmd] = useState('');
  const [hist, setHist] = useState<string[]>([]);
  const [hi, setHi] = useState(-1);
  const box = useRef<HTMLDivElement>(null);
  useEffect(() => { box.current?.scrollTo(0, box.current.scrollHeight); }, [log]);

  const submit = () => {
    const c = cmd.trim(); if (!c) return;
    void send(c); setHist([c, ...hist.slice(0, 49)]); setHi(-1); setCmd('');
  };
  return (
    <section className="panel console">
      <h2>Serial console</h2>
      <div className="log" ref={box}>
        {log.map(l => <div key={l.id} className={`l-${l.dir} ${l.text.startsWith('ERR') ? 'l-err' : ''} ${l.text.startsWith('EV') ? 'l-ev' : ''}`}>
          {l.dir === 'out' ? '> ' : l.dir === 'info' ? '# ' : ''}{l.text}</div>)}
      </div>
      <form className="row" onSubmit={e => { e.preventDefault(); submit(); }}>
        <input className="grow" value={cmd} placeholder="e.g. MOVE 200 0 100 · STATUS · SET grip_wiggle 5" aria-label="Command"
          onChange={e => setCmd(e.target.value)}
          onKeyDown={e => {
            if (e.key === 'ArrowUp' && hist.length) { const i = Math.min(hi + 1, hist.length - 1); setHi(i); setCmd(hist[i]); e.preventDefault(); }
            if (e.key === 'ArrowDown') { const i = hi - 1; setHi(i); setCmd(i >= 0 ? hist[i] : ''); e.preventDefault(); }
          }} />
        <button type="submit">Send</button>
      </form>
    </section>
  );
}
