// Move-to: Cartesian (MOVE / LINE, checked with the TS IK first) and joint space (MOVEJ / GRIP).
import { useState } from 'react';
import { useArm } from '../store/useArm';
import { ik } from '../core/kinematics';
import { fmt } from '../link/protocol';
import { JOINT_MAX, JOINT_MIN, JOINT_NAMES } from '../core/config';

export function MovePanel() {
  const send = useArm(s => s.send);
  const tel = useArm(s => s.tel);
  const [p, setP] = useState({ x: 200, y: 0, z: 100, roll: 0 });
  const [q, setQ] = useState([0, 30, 90, 0, 60, 0, 45]);
  const [msg, setMsg] = useState('');
  const check = ik(p);

  const reply = (r: Awaited<ReturnType<typeof send>>) => setMsg(r ? (r.ok ? `OK ${r.cmd}` : `ERR ${r.cmd} ${r.text}`) : 'no reply');
  const fromArm = () => { if (tel) { setP({ x: tel.tip.x, y: tel.tip.y, z: tel.tip.z, roll: Math.round(tel.joints[5] + tel.joints[0]) }); setQ(tel.joints.map(v => Math.round(v))); } };

  return (
    <section className="panel">
      <h2>Move to</h2>
      <div className="row">
        {(['x', 'y', 'z', 'roll'] as const).map(k => (
          <label key={k} className="num">{k}
            <input type="number" value={p[k]} step={k === 'roll' ? 5 : 10} onChange={e => setP({ ...p, [k]: Number(e.target.value) })} />
          </label>
        ))}
      </div>
      <div className="row">
        <button disabled={check.status !== 'ok'} onClick={async () => reply(await send(fmt.move(p.x, p.y, p.z, p.roll)))}>Move</button>
        <button disabled={check.status !== 'ok'} onClick={async () => reply(await send(fmt.line(p.x, p.y, p.z)))}>Line</button>
        <button className="ghost" onClick={fromArm} disabled={!tel}>← from arm</button>
        {check.status !== 'ok' && <span className="bad-text">{check.status === 'limit' ? 'joint limit' : 'unreachable'}</span>}
        {check.status === 'ok' && <span className="muted small">J1..J3 {check.q.slice(0, 3).map(v => v.toFixed(0)).join(' / ')}°</span>}
      </div>
      <h3>Joints</h3>
      {JOINT_NAMES.map((name, j) => (
        <label key={name} className="slider">
          <span>{name}</span>
          <input type="range" min={JOINT_MIN[j]} max={JOINT_MAX[j]} step={j === 6 ? 1 : 0.5} value={q[j]}
            onChange={e => setQ(q.map((v, k) => (k === j ? Number(e.target.value) : v)))} />
          <output>{q[j].toFixed(j === 6 ? 0 : 1)}{j === 6 ? ' mm' : '°'}</output>
        </label>
      ))}
      <div className="row">
        <button onClick={async () => reply(await send(fmt.movej(q)))}>Send MOVEJ</button>
        <button onClick={async () => reply(await send(fmt.grip(q[6])))}>Send GRIP</button>
        <span className="muted small">{msg}</span>
      </div>
    </section>
  );
}
