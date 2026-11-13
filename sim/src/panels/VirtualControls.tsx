// Virtual mode only: poke the simulated world to exercise the state machine.
import { useState } from 'react';
import { useArm, useVirtualFw } from '../store/useArm';

export function VirtualControls() {
  const fw = useVirtualFw();
  useArm(s => s.tel);                                  // re-render with the world at 20 Hz
  const [, force] = useState(0);
  if (!fw) return null;
  const w = fw.world, poke = (f: () => void) => { f(); force(n => n + 1); };
  return (
    <section className="panel">
      <h2>Virtual world</h2>
      <div className="row">
        <button className="red" disabled={!!w.cube} onClick={() => poke(() => fw.placeCube('RED'))}>Place red cube</button>
        <button className="blue" disabled={!!w.cube} onClick={() => poke(() => fw.placeCube('BLUE'))}>Place blue cube</button>
        <button disabled={w.cube?.where !== 'cradle'} onClick={() => poke(() => fw.removeCube())}>Remove</button>
      </div>
      <div className="row">
        <button className={w.failNextPick ? 'on' : ''} onClick={() => poke(() => fw.failNextPick())}>Next pick fails{w.failNextPick ? ' ✓' : ''}</button>
        <button className={w.powerOk ? 'warn' : 'bad'} onClick={() => poke(() => fw.setPower(!w.powerOk))}>{w.powerOk ? 'Press E-stop' : 'Release E-stop'}</button>
      </div>
      <div className="row">
        <span className="muted small">Break switch wire:</span>
        {[0, 1, 2].map(j => (
          <label key={j} className="check"><input type="checkbox" checked={w.brokenSwitch[j]} onChange={e => poke(() => fw.breakSwitch(j, e.target.checked))} />J{j + 1}</label>
        ))}
      </div>
      <div className="muted small">bins: A (red) {w.bins.A} · B (blue) {w.bins.B} · cube: {w.cube ? `${w.cube.colour} in ${w.cube.where}` : 'none'}</div>
    </section>
  );
}
