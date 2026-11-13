// Teach mode: jog the arm (Cartesian or per joint), save/go poses, colour calibration.
import { useState } from 'react';
import { useArm } from '../store/useArm';
import { fmt, POSE_NAMES } from '../link/protocol';

const STEPS = [10, 1, 0.2];

export function TeachPanel() {
  const { send, tel, poses, refreshPoses } = useArm();
  const [step, setStep] = useState(1);
  const [msg, setMsg] = useState('');
  const teaching = tel?.state === 'TEACH';
  const run = async (cmd: string) => { const r = await send(cmd); setMsg(r ? `${r.ok ? 'OK' : 'ERR'} ${r.cmd} ${r.text}` : 'no reply'); return r; };
  const jog = (dx: number, dy: number, dz: number, dr = 0) => run(fmt.jog(dx * step, dy * step, dz * step, dr * step * 5));

  return (
    <section className="panel">
      <h2>Teach</h2>
      <div className="row">
        {teaching ? <button onClick={() => run('DONE')}>Leave TEACH</button> : <button className="primary" onClick={() => run('TEACH')}>Enter TEACH</button>}
        <div className="seg">{STEPS.map(s => <button key={s} className={s === step ? 'on' : ''} onClick={() => setStep(s)}>{s} mm</button>)}</div>
      </div>
      <div className="jog" aria-disabled={!teaching}>
        <button onClick={() => jog(0, 1, 0)}>Y+</button>
        <button onClick={() => jog(1, 0, 0)}>X+</button>
        <button onClick={() => jog(0, 0, 1)}>Z+</button>
        <button onClick={() => jog(0, 0, 0, 1)}>Roll+</button>
        <button onClick={() => jog(0, -1, 0)}>Y−</button>
        <button onClick={() => jog(-1, 0, 0)}>X−</button>
        <button onClick={() => jog(0, 0, -1)}>Z−</button>
        <button onClick={() => jog(0, 0, 0, -1)}>Roll−</button>
      </div>
      <div className="jogj">
        {['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'Grip'].map((n, j) => (
          <span key={n}>
            <button onClick={() => run(fmt.jogj(j + 1, -step))}>−</button>
            <b>{n}</b>
            <button onClick={() => run(fmt.jogj(j + 1, step))}>+</button>
          </span>
        ))}
      </div>
      <table className="poses">
        <thead><tr><th>Pose</th><th>x</th><th>y</th><th>z</th><th>roll</th><th /></tr></thead>
        <tbody>
          {POSE_NAMES.map(n => {
            const p = poses[n];
            return (
              <tr key={n}>
                <td>{n}</td>
                {p ? [p.x, p.y, p.z, p.roll].map((v, i) => <td key={i}>{v.toFixed(1)}</td>) : <td colSpan={4} className="muted">—</td>}
                <td className="nowrap">
                  <button onClick={async () => { await run(`SAVE ${n}`); await refreshPoses(); }}>Save</button>
                  <button onClick={() => run(`GO ${n}`)}>Go</button>
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
      <div className="row">
        <span className="muted small">Colour cal (cube on sensor):</span>
        {['RED', 'BLUE', 'EMPTY'].map(c => <button key={c} onClick={() => run(`CAL ${c}`)}>CAL {c}</button>)}
      </div>
      <div className="muted small">{teaching ? 'jog = step size; roll jog = 5° × step' : 'Enter TEACH to jog, save and calibrate.'} {msg}</div>
    </section>
  );
}
