// Live joint-angle plot (last 20 s, plain canvas) + tip / IR / colour / limits readout.
import { useEffect, useRef } from 'react';
import { useArm } from '../store/useArm';

const COLORS = ['#e3a33b', '#4fb3ff', '#7bd389', '#c77dff', '#ff6b6b', '#f4d35e'];
const WINDOW = 20000, YMIN = -110, YMAX = 160;

export function TelemetryPanel() {
  const tel = useArm(s => s.tel);
  const history = useArm(s => s.history);
  const ref = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const c = ref.current; if (!c) return;
    const dpr = window.devicePixelRatio || 1, w = c.clientWidth, h = c.clientHeight;
    if (c.width !== w * dpr) { c.width = w * dpr; c.height = h * dpr; }
    const g = c.getContext('2d')!;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);
    const Y = (v: number) => h - ((v - YMIN) / (YMAX - YMIN)) * h;
    g.strokeStyle = '#333a45'; g.fillStyle = '#7d8696'; g.font = '10px system-ui'; g.lineWidth = 1;
    for (const v of [-90, 0, 90]) { g.beginPath(); g.moveTo(0, Y(v)); g.lineTo(w, Y(v)); g.stroke(); g.fillText(`${v}°`, 2, Y(v) - 2); }
    if (!history.length) return;
    const t1 = history[history.length - 1].t, X = (t: number) => w - ((t1 - t) / WINDOW) * w;
    for (let j = 0; j < 6; j++) {
      g.strokeStyle = COLORS[j]; g.lineWidth = 1.5; g.beginPath();
      history.forEach((s, i) => (i ? g.lineTo(X(s.t), Y(s.j[j])) : g.moveTo(X(s.t), Y(s.j[j]))));
      g.stroke();
    }
  }, [history]);

  const lim = tel?.limits ?? [false, false, false];
  return (
    <section className="panel telemetry">
      <h2>Telemetry</h2>
      <div className="legend">{COLORS.map((c, j) => <span key={j}><i style={{ background: c }} />J{j + 1} {tel ? tel.joints[j].toFixed(1) : '—'}°</span>)}
        <span>grip {tel ? tel.joints[6].toFixed(1) : '—'} mm</span></div>
      <canvas ref={ref} className="plot" aria-label="Joint angles, last 20 seconds" />
      <div className="readout">
        <span>tip <b>{tel ? `${tel.tip.x}, ${tel.tip.y}, ${tel.tip.z}` : '—'}</b> mm</span>
        <span>IR <b className={tel?.ir ? 'ok-text' : ''}>{tel?.ir ? 'blocked' : 'clear'}</b></span>
        <span>colour <b className={`c-${tel?.colour}`}>{tel?.colour ?? '—'}</b></span>
        <span>limits {lim.map((l, i) => <b key={i} className={l ? 'warn-text' : 'muted'}>J{i + 1} </b>)}</span>
      </div>
    </section>
  );
}
