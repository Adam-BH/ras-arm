// Teach mode: jog the arm (Cartesian or per joint), save/go/preview stations, add a station here,
// colour → bin map (live on the arm), colour calibration.
import { useState } from 'react';
import { useArm } from '../store/useArm';
import { fmt } from '../link/protocol';
import { MAP_COLOURS, STATION_TYPES, type StationType } from '../core/stations';

const STEPS = [10, 1, 0.2];

export function TeachPanel() {
  const { send, tel, layout, loadLayout } = useArm();
  const [step, setStep] = useState(1);
  const [newName, setNewName] = useState('REJECT');
  const [newType, setNewType] = useState<StationType>('BIN');
  const [msg, setMsg] = useState('');
  const teaching = tel?.state === 'TEACH';
  const run = async (cmd: string) => { const r = await send(cmd); setMsg(r ? `${r.ok ? 'OK' : 'ERR'} ${r.cmd} ${r.text}` : 'no reply'); return r; };
  const runAndReload = async (cmd: string) => { const r = await run(cmd); await loadLayout(); return r; };
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
        <thead><tr><th>Station</th><th>x</th><th>y</th><th>z</th><th>roll</th><th /></tr></thead>
        <tbody>
          {layout.stations.map(p => (
            <tr key={p.name}>
              <td title={p.type}>{p.name}</td>
              {[p.x, p.y, p.z, p.roll].map((v, i) => <td key={i}>{v.toFixed(0)}</td>)}
              <td className="nowrap">
                <button title="save the current tip pose here (validated, committed)" onClick={() => runAndReload(`SAVE ${p.name}`)}>Save</button>
                <button title="straight to the grip pose" onClick={() => run(`GO ${p.name}`)}>Go</button>
                <button title="above the station, then a slow line down" onClick={() => run(`ST GO ${p.name}`)}>Preview</button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      <div className="row">
        <input aria-label="new station name" value={newName} maxLength={7} size={8} onChange={e => setNewName(e.target.value.toUpperCase())} />
        <select aria-label="new station type" value={newType} onChange={e => setNewType(e.target.value as StationType)}>
          {STATION_TYPES.map(t => <option key={t}>{t}</option>)}
        </select>
        <button onClick={() => runAndReload(`ST ADD ${newName} ${newType}`)}>Add station here</button>
        <button onClick={() => runAndReload('ST COMMIT')}>Commit</button>
      </div>
      <div className="row">
        {MAP_COLOURS.map(c => (
          <label key={c} className="num">{c} →
            <select value={layout.map[c] ?? '-'} onChange={e => runAndReload(fmt.stMap(c, e.target.value === '-' ? null : e.target.value))}>
              <option value="-">—</option>
              {layout.stations.filter(s => s.type === 'BIN').map(s => <option key={s.name}>{s.name}</option>)}
            </select>
          </label>
        ))}
      </div>
      <div className="row">
        <span className="muted small">Colour cal (cube on sensor):</span>
        {['RED', 'BLUE', 'EMPTY'].map(c => <button key={c} onClick={() => run(`CAL ${c}`)}>CAL {c}</button>)}
      </div>
      <div className="muted small">{teaching ? 'jog = step size; roll jog = 5° × step' : 'Enter TEACH to jog, save and calibrate.'} {msg}</div>
    </section>
  );
}
