// Vision: the overhead camera live, what it finds (cube outlines projected back on the image, rejected
// blobs dashed), loose cubes to scatter (virtual mode) and the vision sort run.
import { useEffect, useRef, useState } from 'react';
import { CELL } from '../core/config';
import { useArm, useVirtualFw } from '../store/useArm';
import { PIXEL_TO_TABLE, useVision } from '../store/useVision';
import { CAMERA, ZONE } from '../vision/config';
import { classMap, type Detection, type Frame } from '../vision/detect';
import { applyH, invertH, type Vec2 } from '../vision/geometry';
import { graspRoll, scatterLayout } from '../vision/plan';

const TABLE_TO_PIXEL = invertH(PIXEL_TO_TABLE);
const LIVE_MS = 200;
const COL = { RED: '#ff5a5a', BLUE: '#5a9bff' };

const toPx = (x: number, y: number) => applyH(TABLE_TO_PIXEL, [x, y]);
function square(d: Detection, grow = 0): Vec2[] {
  const h = CELL.cubeSize / 2 + grow, a = (d.yaw * Math.PI) / 180, c = Math.cos(a), s = Math.sin(a);
  return [[-h, -h], [h, -h], [h, h], [-h, h]].map(([u, v]) => toPx(d.x + c * u - s * v, d.y + s * u + c * v));
}

function draw(g: CanvasRenderingContext2D, f: Frame, found: Detection[], mask: boolean, target: Detection | null) {
  let px = f.data;
  if (mask) {                                         // the HSV classes over a dimmed picture
    const cls = classMap(f);
    px = new Uint8ClampedArray(f.data.length);
    for (let i = 0; i < cls.length; i++) {
      const k = i * 4, rgb = cls[i] === 1 ? [255, 70, 70] : cls[i] === 2 ? [70, 140, 255] : [f.data[k] / 3, f.data[k + 1] / 3, f.data[k + 2] / 3];
      px[k] = rgb[0]; px[k + 1] = rgb[1]; px[k + 2] = rgb[2]; px[k + 3] = 255;
    }
  }
  g.putImageData(new ImageData(px, f.width, f.height), 0, 0);
  const path = (pts: Vec2[]) => { g.beginPath(); pts.forEach(([u, v], i) => (i ? g.lineTo(u, v) : g.moveTo(u, v))); g.closePath(); };
  g.lineWidth = 1; g.setLineDash([5, 4]); g.strokeStyle = '#e3a33b';
  path([[ZONE.xMin, ZONE.yMin], [ZONE.xMax, ZONE.yMin], [ZONE.xMax, ZONE.yMax], [ZONE.xMin, ZONE.yMax]].map(([x, y]) => toPx(x, y)));
  g.stroke();
  g.font = '11px system-ui'; g.textBaseline = 'bottom';
  for (const d of found) {
    const [u, v] = toPx(d.x, d.y);
    if (d.reject) {                                   // why it is not a cube: shown with the mask
      if (!mask) continue;
      g.setLineDash([2, 3]); g.strokeStyle = '#c9ced6'; g.lineWidth = 1;
      g.strokeRect(u - 14, v - 14, 28, 28);
      g.fillStyle = '#c9ced6'; g.fillText(d.reject, u - 14, v - 16);
      continue;
    }
    g.setLineDash([]); g.strokeStyle = COL[d.colour]; g.lineWidth = 2;
    path(square(d)); g.stroke();
    g.beginPath(); g.moveTo(u - 4, v); g.lineTo(u + 4, v); g.moveTo(u, v - 4); g.lineTo(u, v + 4); g.stroke();
    const label = `${d.x.toFixed(0)}, ${d.y.toFixed(0)} · ${d.yaw.toFixed(0)}°`, tw = g.measureText(label).width + 6;
    const lx = u + 14 + tw > f.width ? u - 14 - tw : u + 14;   // flip left at the right edge
    g.fillStyle = 'rgba(16,19,23,.75)'; g.fillRect(lx, v - 26, tw, 15);
    g.fillStyle = '#e2e6ec'; g.fillText(label, lx + 3, v - 12);
  }
  if (target) {
    g.setLineDash([]); g.strokeStyle = '#ffd23f'; g.lineWidth = 2;
    path(square(target, 7)); g.stroke();
  }
}

export function VisionPanel() {
  const { frame, detections, cameraError, sorter, sort, look, startSort, stopSort } = useVision();
  const fw = useVirtualFw();
  const tel = useArm(s => s.tel), connected = useArm(s => s.connected), mode = useArm(s => s.mode);
  const gripZ = useArm(s => s.poses.CUBE?.z ?? CELL.cube.z);
  const [mask, setMask] = useState(false);
  const ref = useRef<HTMLCanvasElement>(null);

  // live view (paused while a sort run takes its own pictures)
  useEffect(() => {
    if (sorter) return;
    let on = true;
    const loop = async () => {
      if (!on) return;
      if (!document.hidden) await look();
      setTimeout(loop, LIVE_MS);
    };
    void loop();
    return () => { on = false; };
  }, [sorter, look]);

  useEffect(() => {
    const g = ref.current?.getContext('2d');
    if (g && frame) draw(g, frame, detections, mask, sorter ? sort?.target ?? null : null);
  }, [frame, detections, mask, sorter, sort?.target]);

  const cubes = detections.filter(d => !d.reject);
  const running = !!sorter;
  const canSort = mode === 'virtual' && connected && tel?.state === 'IDLE' && !running;
  const scatter = () => fw?.scatter(scatterLayout(Array.from({ length: 4 }, () => (Math.random() < 0.5 ? 'RED' : 'BLUE'))));
  const secs = sort ? (((sort.t1 ?? Date.now()) - sort.t0) / 1000).toFixed(1) : '';

  return (
    <section className="panel vision">
      <h2>Vision · overhead camera</h2>
      <div className="vision-body">
        <div className="cam">
          <canvas ref={ref} width={CAMERA.width} height={CAMERA.height} aria-label="Overhead camera with detected cubes" />
          {cameraError && <div className="small warn-text">{cameraError}</div>}
        </div>
        <div className="side">
          <div className="row">
            {fw && <button disabled={running} onClick={scatter}>Scatter 4 cubes</button>}
            {fw && <button className="ghost" disabled={running || !fw.world.table.length} onClick={() => fw.clearTable()}>Clear table</button>}
            <label className="check"><input type="checkbox" checked={mask} onChange={e => setMask(e.target.checked)} />HSV mask</label>
          </div>
          <div className="row">
            {running
              ? <button className="bad" onClick={stopSort}>Stop sort</button>
              : <button className="primary" disabled={!canSort} onClick={startSort}>Sort by vision</button>}
            {mode === 'usb' && <span className="muted small">USB: needs a calibrated webcam (next step)</span>}
            {mode === 'virtual' && !running && connected && tel?.state !== 'IDLE' && <span className="muted small">waits for IDLE</span>}
          </div>
          {sort && (
            <div className="sort-status small">
              <span className={`badge ph-${sort.phase}`}>{sort.phase}</span> {sort.text}
              <div className="muted">sorted <b>{sort.sorted}</b> · bin A {sort.bins.A} · bin B {sort.bins.B} · slips {sort.slips} · {secs} s</div>
            </div>
          )}
          <table className="poses dets">
            <thead><tr><th>cube</th><th>x</th><th>y</th><th>yaw</th><th>reach</th></tr></thead>
            <tbody>
              {cubes.map((d, i) => (
                <tr key={i}>
                  <td className={`c-${d.colour}`}>● {d.colour.toLowerCase()}</td>
                  <td>{d.x.toFixed(0)}</td><td>{d.y.toFixed(0)}</td><td>{d.yaw.toFixed(0)}°</td>
                  <td>{graspRoll(d.x, d.y, d.yaw, gripZ) !== null ? <span className="ok-text">ok</span> : <span className="warn-text">out</span>}</td>
                </tr>
              ))}
              {!cubes.length && <tr><td colSpan={5} className="muted">no cube in the zone</td></tr>}
            </tbody>
          </table>
          <div className="muted small">HSV threshold → blobs → pixels mapped to the table → centre + yaw (4-fold moments) · mm, robot frame</div>
        </div>
      </div>
    </section>
  );
}
