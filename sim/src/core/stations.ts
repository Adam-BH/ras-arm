// Port of firmware/src/4_motion/stations.c — the workcell as data: named stations + colour → bin map.
// validate() follows the same rules in the same order (tested on the same golden vectors as the C code),
// so the layout editor's reach shading shows exactly what the arm will accept.
import { CELL, J1_WORK_MAX, J1_WORK_MIN, LINE, SAFE_Z, STATION, IK_MAX_COS3, L1, L2, L3, H_SHOULDER } from './config';
import { ik, type Pose } from './kinematics';

export const STATION_TYPES = ['PARK', 'PICK', 'BIN'] as const;
export type StationType = (typeof STATION_TYPES)[number];
export const MAP_COLOURS = ['RED', 'BLUE', 'UNKNOWN'] as const;
export type MapColour = (typeof MAP_COLOURS)[number];
export const ST_ERRORS = ['OK', 'UNREACHABLE', 'SINGULAR', 'LIMIT', 'KEEPOUT', 'TOO_CLOSE', 'FULL', 'NAME', 'NOT_TEACH', 'PROTECTED'] as const;
export type StError = (typeof ST_ERRORS)[number];

export type Station = { name: string; type: StationType; x: number; y: number; z: number; roll: number; aboveDz: number };
export type Layout = { stations: Station[]; map: Record<MapColour, string | null> };   // map: colour → bin name

export function cadLayout(): Layout {
  const d = STATION.aboveDz;
  return {
    stations: [
      { name: 'HOME', type: 'PARK', ...CELL.home, aboveDz: d },
      { name: 'CUBE', type: 'PICK', ...CELL.cube, aboveDz: d },
      { name: 'BIN_A', type: 'BIN', ...CELL.binA, aboveDz: d },
      { name: 'BIN_B', type: 'BIN', ...CELL.binB, aboveDz: d },
    ],
    map: { RED: 'BIN_A', BLUE: 'BIN_B', UNKNOWN: null },
  };
}

/** approach height: max(grip + above_dz, SAFE_Z); park poses have no approach */
export const aboveZ = (s: { type: StationType; z: number; aboveDz: number }) =>
  s.type === 'PARK' ? s.z : Math.max(s.z + s.aboveDz, SAFE_Z);

const cos3 = (p: Pose) => {
  const r = Math.hypot(p.x, p.y), zw = p.z + L3 - H_SHOULDER;
  return (r * r + zw * zw - L1 * L1 - L2 * L2) / (2 * L1 * L2);
};

function check(p: Pose, type: StationType): StError {
  if (p.z < STATION.zMinTable || Math.hypot(p.x, p.y) < STATION.rMinBase) return 'KEEPOUT';
  const r = ik(p);
  if (r.status === 'unreachable') { const c = cos3(p); return c > IK_MAX_COS3 && c <= 1 ? 'SINGULAR' : 'UNREACHABLE'; }
  if (r.status === 'limit') return 'LIMIT';
  if (type !== 'PARK' && (r.q[0] < J1_WORK_MIN || r.q[0] > J1_WORK_MAX)) return 'LIMIT';
  return 'OK';
}

/** like stations_validate(): grip pose, the vertical line up to the approach, spacing to the other stations */
export function validate(p: Pose, aboveDz: number, type: StationType, stations: Station[], self = -1): StError {
  if (aboveDz < 0 || (type === 'BIN' && p.z < STATION.binMinDropZ)) return 'KEEPOUT';
  const top = aboveZ({ type, z: p.z, aboveDz });
  for (let z = p.z; ; z = Math.min(z + LINE.stepMm, top)) {
    const e = check({ ...p, z }, type);
    if (e !== 'OK') return e;
    if (z >= top) break;
  }
  if (type === 'PARK') return 'OK';
  for (let i = 0; i < stations.length; i++) {
    const s = stations[i];
    if (i === self || s.type === 'PARK') continue;
    if (Math.hypot(p.x - s.x, p.y - s.y) < STATION.minGap) return 'TOO_CLOSE';
  }
  return 'OK';
}

export const nameOk = (n: string) => new RegExp(`^[A-Z0-9_]{1,${STATION.nameLen}}$`).test(n);
const q10 = (v: number) => Math.round(v * 10) / 10;            // the firmware stores 0.1 mm / 0.1°

/** the firmware's table: RAM edits, commit/revert/factory, same invariants (HOME + pick station protected) */
export class StationTable {
  layout = cadLayout();
  version = 0;                     // +1 per commit (telemetry)
  dirty = false;
  private saved: Layout | null = null;

  find(name: string) { return this.layout.stations.findIndex(s => s.name === name); }
  pick() { return this.layout.stations.find(s => s.type === 'PICK')!; }
  binFor(c: MapColour) { const n = this.layout.map[c]; return n ? this.layout.stations[this.find(n)] ?? null : null; }

  add(name: string, type: StationType, p: Pose, aboveDz = STATION.aboveDz): StError {
    const st = this.layout.stations;
    if (!nameOk(name) || this.find(name) >= 0) return 'NAME';
    if (st.length >= STATION.max || (type === 'PICK' && st.some(s => s.type === 'PICK'))) return 'FULL';
    const s: Station = { name, type, x: q10(p.x), y: q10(p.y), z: q10(p.z), roll: q10(p.roll), aboveDz: Math.trunc(aboveDz) };
    const e = validate(s, s.aboveDz, type, st);
    if (e === 'OK') { st.push(s); this.dirty = true; }
    return e;
  }
  set(i: number, p: Pose, aboveDz: number): StError {
    const s = this.layout.stations[i];
    const n = { ...s, x: q10(p.x), y: q10(p.y), z: q10(p.z), roll: q10(p.roll), aboveDz: Math.trunc(aboveDz) };
    const e = validate(n, n.aboveDz, s.type, this.layout.stations, i);
    if (e === 'OK') { this.layout.stations[i] = n; this.dirty = true; }
    return e;
  }
  del(i: number): StError {
    const s = this.layout.stations[i];
    if (i === 0 || s === this.pick()) return 'PROTECTED';
    this.layout.stations.splice(i, 1);
    for (const c of MAP_COLOURS) if (this.layout.map[c] === s.name) this.layout.map[c] = null;
    this.dirty = true;
    return 'OK';
  }
  map(c: MapColour, name: string | null): StError {
    if (name !== null && this.layout.stations[this.find(name)]?.type !== 'BIN') return 'NAME';
    this.layout.map[c] = name; this.dirty = true;
    return 'OK';
  }
  commit() { this.version = (this.version + 1) & 0xff; this.saved = structuredClone(this.layout); this.dirty = false; }
  revert() { this.layout = this.saved ? structuredClone(this.saved) : cadLayout(); this.dirty = false; }
  factory() { this.layout = cadLayout(); this.dirty = true; }
}
