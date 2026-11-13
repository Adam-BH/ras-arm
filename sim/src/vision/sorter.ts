// Vision sort, host side: look → pick the nearest cube → look again to check it left the table →
// drop it in its colour's bin → repeat until the camera sees no reachable cube.
// Only the normal protocol (STATUS, MOVE, GRIP, STOP): the firmware needs nothing new, and the same
// loop drives the virtual arm or the real one (with a calibrated webcam as the frame source).
import { CELL, GRIP_CLOSED_MM, GRIP_OPEN_MM, PLACE_ABOVE_DZ, SAFE_Z } from '../core/config';
import type { Pose } from '../core/kinematics';
import { commandName, fmt, type Reply } from '../link/protocol';
import { JIGGLE, SORT, VIEW_POSE } from './config';
import { detect, type CubeColour, type Detection, type Frame } from './detect';
import type { Mat3 } from './geometry';
import { graspRoll } from './plan';

export type SorterIo = {
  send(cmd: string): Promise<Reply | null>;
  onEvent(cb: (line: string) => void): () => void;   // "EV …" / "DONE …" lines; returns unsubscribe
  capture(): Promise<Frame>;
  pixelToTable: Mat3;
  poses(): Record<string, Pose>;                       // taught CUBE (grip height), BIN_A, BIN_B
  wait(ms: number): Promise<void>;
};

export type Phase = 'idle' | 'view' | 'look' | 'pick' | 'verify' | 'place' | 'done' | 'stopped' | 'error';
export type SortStatus = {
  phase: Phase; text: string;
  seen: Detection[];                                   // last snapshot, rejected blobs included
  target: Detection | null; attempt: number;
  sorted: number; bins: { A: number; B: number }; slips: number; skipped: number;
  t0: number; t1?: number;                             // wall-clock ms
};

const DONE_TIMEOUT_MS = 20000;
const dist = (a: { x: number; y: number }, b: { x: number; y: number }) => Math.hypot(a.x - b.x, a.y - b.y);

export class VisionSorter {
  status: SortStatus = { phase: 'idle', text: '', seen: [], target: null, attempt: 0, sorted: 0, bins: { A: 0, B: 0 }, slips: 0, skipped: 0, t0: Date.now() };
  private stopped = false;
  private cancel: ((e: Error) => void) | null = null;

  constructor(private io: SorterIo, private onStatus: (s: SortStatus) => void = () => {}) {}

  stop() {
    if (this.stopped) return;
    this.stopped = true;
    this.cancel?.(new Error('stopped'));
    void this.io.send('STOP');
  }

  private set(p: Partial<SortStatus>) { this.status = { ...this.status, ...p }; this.onStatus(this.status); }

  async run(): Promise<SortStatus> {
    this.set({ phase: 'view', text: 'checking the arm', t0: Date.now() });
    try {
      const st = await this.io.send('STATUS');
      if (!st?.ok || !st.text.startsWith('IDLE ') || st.text.includes('running=1'))
        throw new Error(`the arm must be IDLE, homed and not running (${st?.text || 'no reply'})`);
      const gripZ = this.io.poses().CUBE?.z ?? CELL.cube.z;    // the taught grip height
      const skip: Detection[] = [];
      for (let picks = 0; picks < SORT.maxPicks; picks++) {
        await this.goView();
        this.set({ phase: 'look', text: 'looking', target: null, attempt: 0 });
        const cubes = (await this.look()).filter(d => !skip.some(s => s.colour === d.colour && dist(s, d) < SORT.verifyMm));
        const todo = cubes.filter(d => graspRoll(d.x, d.y, d.yaw, gripZ) !== null)
          .sort((a, b) => Math.hypot(a.x, a.y) - Math.hypot(b.x, b.y));
        if (!todo.length) {
          await this.exec(fmt.grip(GRIP_OPEN_MM));
          const out = cubes.length, why = [out && `${out} out of reach`, skip.length && `${skip.length} given up`].filter(Boolean);
          this.set({ phase: 'done', text: `table clear${why.length ? ` (${why.join(', ')})` : ''}`, t1: Date.now() });
          return this.status;
        }
        if (await this.pickVerified(todo[0], gripZ)) await this.place(todo[0].colour);
        else { skip.push(todo[0]); this.set({ skipped: skip.length }); }
      }
      throw new Error(`gave up after ${SORT.maxPicks} picks`);
    } catch (e) {
      this.set(this.stopped ? { phase: 'stopped', text: 'stopped', t1: Date.now() } : { phase: 'error', text: (e as Error).message, t1: Date.now() });
    }
    return this.status;
  }

  /** pick, move out of the picture, look: still there → it slipped, nudge and retry */
  private async pickVerified(first: Detection, gripZ: number): Promise<boolean> {
    let target = first;
    for (let attempt = 0; attempt <= SORT.retries; attempt++) {
      const [dx, dy, dr] = JIGGLE[attempt % JIGGLE.length];
      const roll = graspRoll(target.x + dx, target.y + dy, target.yaw + dr, gripZ);
      if (roll === null) return false;
      const where = `${target.colour.toLowerCase()} cube at ${target.x.toFixed(0)}, ${target.y.toFixed(0)}`;
      this.set({ phase: 'pick', text: `picking the ${where}${attempt ? ` (retry ${attempt})` : ''}`, target, attempt });
      await this.pick(target.x + dx, target.y + dy, gripZ, roll);
      await this.goView();
      this.set({ phase: 'verify', text: 'checking the pick' });
      const still = (await this.look()).find(d => d.colour === target.colour && dist(d, target) < SORT.verifyMm);
      if (!still) return true;
      this.set({ slips: this.status.slips + 1 });
      target = still;                                  // re-aim at where the cube is now
    }
    return false;
  }

  private async pick(x: number, y: number, z: number, roll: number) {
    await this.exec(fmt.grip(GRIP_OPEN_MM));
    await this.exec(fmt.move(x, y, SAFE_Z, roll));
    await this.exec(fmt.move(x, y, z, roll, SORT.descendMs));
    await this.exec(fmt.grip(GRIP_CLOSED_MM));
    await this.exec(fmt.move(x, y, SAFE_Z, roll, SORT.liftMs));
  }

  private async place(colour: CubeColour) {
    const bin = colour === 'RED' ? 'A' : 'B';
    const p = this.io.poses()[`BIN_${bin}`] ?? (bin === 'A' ? CELL.binA : CELL.binB), up = SAFE_Z + PLACE_ABOVE_DZ;
    this.set({ phase: 'place', text: `dropping it in bin ${bin}` });
    await this.exec(fmt.move(p.x, p.y, up, p.roll));
    await this.exec(fmt.move(p.x, p.y, p.z, p.roll, SORT.placeMs));
    await this.exec(fmt.grip(GRIP_OPEN_MM));
    await this.exec(fmt.move(p.x, p.y, up, p.roll, SORT.placeMs));
    this.set({ sorted: this.status.sorted + 1, bins: { ...this.status.bins, [bin]: this.status.bins[bin] + 1 } });
  }

  private goView() {
    this.set({ phase: 'view', text: 'moving out of the picture' });
    return this.exec(fmt.move(VIEW_POSE.x, VIEW_POSE.y, VIEW_POSE.z, VIEW_POSE.roll));
  }

  private async look(): Promise<Detection[]> {
    await this.io.wait(SORT.settleMs);
    if (this.stopped) throw new Error('stopped');
    const seen = detect(await this.io.capture(), this.io.pixelToTable);
    this.set({ seen });
    return seen.filter(d => !d.reject);
  }

  /** send a motion command, wait for its DONE (or an ERROR / ESTOP event) */
  private async exec(cmd: string) {
    if (this.stopped) throw new Error('stopped');
    const name = commandName(cmd);
    let off = () => {}, timer: ReturnType<typeof setTimeout> | undefined, end!: (e?: Error) => void;
    const done = new Promise<void>((resolve, reject) => {
      end = e => {
        off(); clearTimeout(timer);
        if (this.cancel === end) this.cancel = null;
        if (e) reject(e); else resolve();
      };
      this.cancel = end;
      timer = setTimeout(() => end(new Error(`no DONE ${name}`)), DONE_TIMEOUT_MS);
      off = this.io.onEvent(line => {                  // listen before sending: DONE can follow OK closely
        if (line === `DONE ${name}`) end();
        else if (line.startsWith('EV ERROR') || line === 'EV ESTOP') end(new Error(line.slice(3).toLowerCase()));
      });
    });
    done.catch(() => {});                              // awaited below; no unhandled rejection meanwhile
    const r = await this.io.send(cmd);
    if (!r?.ok) end(new Error(r ? `${cmd} → ${r.text}` : `${cmd}: no reply`));
    await done;
  }
}
