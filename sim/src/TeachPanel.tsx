// sim/src/TeachPanel.tsx — teach mode UI: jog the real arm, save CUBE / BIN_A / BIN_B.
// The 3D twin (Arm3D) follows the telemetry, so you see the arm move on screen too.
import { useEffect, useRef, useState } from 'react';
import { SerialLink, Telemetry } from './serialLink';
import { Arm3D } from './Arm3D';                 // reuses our three.js model

const STEPS = [10, 1, 0.2];                       // mm per click

export function TeachPanel() {
  const link = useRef(new SerialLink());
  const [t, setT] = useState<Telemetry | null>(null);
  const [step, setStep] = useState(1);
  const [log, setLog] = useState<string[]>([]);

  useEffect(() => {
    link.current.onTelemetry = setT;
    link.current.onReply = line => setLog(l => [...l.slice(-30), line]);
  }, []);

  const send = (c: string) => { link.current.send(c); setLog(l => [...l.slice(-30), '> ' + c]); };
  const jog = (dx: number, dy: number, dz: number) => send(`JOG ${dx * step} ${dy * step} ${dz * step}`);

  return (
    <div className="teach">
      <button onClick={() => link.current.connect()}>Connect arm (USB)</button>
      <button onClick={() => send('TEACH')}>Enter TEACH</button>
      <button onClick={() => send('DONE')}>Done</button>

      <div className="jog">
        <button onClick={() => jog(1, 0, 0)}>X+</button><button onClick={() => jog(-1, 0, 0)}>X−</button>
        <button onClick={() => jog(0, 1, 0)}>Y+</button><button onClick={() => jog(0, -1, 0)}>Y−</button>
        <button onClick={() => jog(0, 0, 1)}>Z+</button><button onClick={() => jog(0, 0, -1)}>Z−</button>
        {STEPS.map(s => <button key={s} className={s === step ? 'on' : ''} onClick={() => setStep(s)}>{s} mm</button>)}
      </div>

      <div className="save">
        {['CUBE', 'BIN_A', 'BIN_B', 'HOME'].map(p => (
          <span key={p}><button onClick={() => send(`SAVE ${p}`)}>Save {p}</button>
                        <button onClick={() => send(`GO ${p}`)}>Go</button></span>
        ))}
      </div>

      <p>State: <b>{t?.state ?? '—'}</b> · tip {t ? `${t.tip.x}, ${t.tip.y}, ${t.tip.z} mm` : '—'} · IR {t?.ir ? 'cube' : 'clear'} · colour {t?.colour}</p>
      <Arm3D joints={t?.joints ?? [0, 10, 110, 0, 60, 0, 45]} />
      <pre>{log.join('\n')}</pre>
    </div>
  );
}
