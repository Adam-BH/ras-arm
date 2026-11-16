// A TypeScript twin of the firmware: same protocol, same 11-state machine, same kinematics,
// running against a small simulated world (joints with speed limits, limit switches, IR beam,
// colour sensor, a cube, two bins). Port of firmware/src/{5_tasks,6_app}; keep them in sync.
import {
  BASE_MAX_ACC_DPS2, CELL, COLOUR_TRIES, FW_VERSION, GRIP_CLOSED_MM, GRIP_OPEN_MM, HOME_DIR, HOME_SWITCH_DEG, HOMING,
  JOINT_MAX, JOINT_MAX_DPS, JOINT_MIN, LINE, MOVE, PICK_RETRIES, PLACE_ABOVE_DZ, RAMP, REHOME_EVERY,
  SAFE_Z, TIMING,
} from './config';
import { fk, ik, withinLimits, type Pose } from './kinematics';
import { COLOURS, ERRORS, POSE_NAMES, STATES, type Colour, type StateName } from '../link/protocol';

type CubeColour = 'RED' | 'BLUE';
export type World = {
  cube: { colour: CubeColour; where: 'cradle' | 'gripper' } | null;
  failNextPick: boolean;        // the next grasp slips: cube stays in the cradle
  powerOk: boolean;             // false = E-stop pressed
  brokenSwitch: boolean[];      // NC wire cut → reads "pressed" forever
  bins: { A: number; B: number };
};

type Rgbc = { r: number; g: number; b: number; c: number };
const SENSOR: Record<'RED' | 'BLUE' | 'EMPTY', Rgbc> = {
  RED: { r: 620, g: 180, b: 150, c: 950 }, BLUE: { r: 140, g: 260, b: 560, c: 900 }, EMPTY: { r: 60, g: 65, b: 70, c: 200 },
};
const TICK_MS = 5;
const clamp = (j: number, v: number) => Math.min(JOINT_MAX[j], Math.max(JOINT_MIN[j], v));
const REASON = { state: 'state', busy: 'busy', running: 'running', unreachable: 'unreachable', limit: 'limit', estop: 'estop' } as const;
type Why = keyof typeof REASON | null;

// trapezoid position profile, 25 % accel / 50 % cruise / 25 % decel (same as the stepper ramp)
function trapezoid(k: number) {
  const v = 1 / (1 - RAMP);
  if (k <= 0) return 0;
  if (k >= 1) return 1;
  if (k < RAMP) return 0.5 * v * k * k / RAMP;
  if (k < 1 - RAMP) return 0.5 * v * RAMP + v * (k - RAMP);
  return 1 - 0.5 * v * (1 - k) * (1 - k) / RAMP;
}

export class VirtualFirmware {
  now = 0;
  q = [20, 40, 90, 0, 0, 0, GRIP_OPEN_MM];       // true joint values (arm left in an unknown pose)
  world: World = { cube: null, failNextPick: false, powerOk: true, brokenSwitch: [false, false, false], bins: { A: 0, B: 0 } };
  private out: (line: string) => void;

  constructor(out: (line: string) => void) { this.out = out; this.boot(); }

  // ───────────── motion ─────────────
  private mv: { from: number[]; to: number[]; t0: number; T: number } | null = null;
  private run: { j: number; v: number } | null = null;
  private wp: Pose[] = []; private wpMs = 0;

  private startMove(target: number[], ms = 0) {
    const to = target.map((v, j) => clamp(j, v));
    let T = ms;
    for (let j = 0; j < 7; j++) T = Math.max(T, Math.abs(to[j] - this.q[j]) / JOINT_MAX_DPS[j] / (1 - RAMP) * 1000);
    T = Math.max(T, Math.sqrt(Math.abs(to[0] - this.q[0]) / (RAMP * (1 - RAMP) * BASE_MAX_ACC_DPS2)) * 1000);   // J1 accel cap
    this.run = null;
    this.mv = { from: this.q.slice(), to, t0: this.now, T: Math.max(T, 50) };
  }
  private targets() { return this.mv ? this.mv.to.slice() : this.q.slice(); }
  private busy() { return !!this.mv || !!this.run || this.wp.length > 0; }
  private stop() { this.mv = null; this.run = null; this.wp = []; }
  where(): Pose { return fk(this.q); }

  private moveTo(p: Pose, ms = 0): boolean {
    const r = ik(p);
    if (r.status !== 'ok') return false;
    const t = this.targets();
    this.wp = [];
    this.startMove([...r.q, t[6]], ms);
    return true;
  }
  private moveJoints(q6: number[], ms = 0): boolean {
    if (!withinLimits(q6)) return false;
    this.wp = [];
    this.startMove([...q6.slice(0, 6), this.targets()[6]], ms);
    return true;
  }
  private moveLine(to: Pose, mmS = LINE.defaultMmS): boolean {
    const from = this.where();
    const d = Math.hypot(to.x - from.x, to.y - from.y, to.z - from.z);
    const n = Math.min(LINE.maxWp, Math.max(1, Math.ceil(d / LINE.stepMm)));
    const pts: Pose[] = [];
    for (let k = 1; k <= n; k++) {
      const t = k / n;
      const p = { x: from.x + (to.x - from.x) * t, y: from.y + (to.y - from.y) * t, z: from.z + (to.z - from.z) * t, roll: from.roll + (to.roll - from.roll) * t };
      if (ik(p).status !== 'ok') return false;           // whole line checked before moving
      pts.push(p);
    }
    this.wp = pts; this.wpMs = 1000 * d / n / (mmS > 0 ? mmS : LINE.defaultMmS);
    return true;
  }
  private grip(mm: number, ms = MOVE.grip) { const t = this.targets(); t[6] = mm; this.startMove(t, ms); }
  private joint(j: number, v: number, ms: number) { const t = this.targets(); t[j] = v; this.startMove(t, ms); }

  private updateMotion(dt: number) {
    if (this.run) this.q[this.run.j] += this.run.v * dt / 1000;
    if (this.mv) {
      const k = (this.now - this.mv.t0) / this.mv.T, s = trapezoid(k);
      for (let j = 0; j < 7; j++) this.q[j] = this.mv.from[j] + (this.mv.to[j] - this.mv.from[j]) * s;
      if (k >= 1) this.mv = null;
    }
    if (!this.mv && !this.run && this.wp.length) {
      const p = this.wp.shift()!, r = ik(p);
      this.startMove([...r.q, this.targets()[6]], this.wpMs);
    }
  }

  // ───────────── sensors ─────────────
  private irRawSince = 0; private irStable = false;
  private limit(j: number) {
    if (this.world.brokenSwitch[j]) return true;
    return HOME_DIR[j] < 0 ? this.q[j] <= HOME_SWITCH_DEG[j] : this.q[j] >= HOME_SWITCH_DEG[j];
  }
  limits(): [boolean, boolean, boolean] { return [this.limit(0), this.limit(1), this.limit(2)]; }
  private estop() { return !this.world.powerOk; }
  private updateSensors() {
    const raw = this.world.cube?.where === 'cradle';
    if (raw !== this.irStable) {
      if (!this.irRawSince) this.irRawSince = this.now;
      if (this.now - this.irRawSince >= (raw ? TIMING.irStable : 20)) { this.irStable = raw; this.irRawSince = 0; }
    } else this.irRawSince = 0;
  }
  ir() { return this.irStable; }
  private reading(): Rgbc { return this.world.cube?.where === 'cradle' ? SENSOR[this.world.cube.colour] : SENSOR.EMPTY; }

  // ───────────── decisions (port of decisions.c) ─────────────
  params: Record<string, number> = {
    confident_ratio: 0.67, same_as_last: 0.08, empty_ratio: 1.3, use_last_trick: 1, grip_wiggle: 5,
    jiggle_mm: 3, jiggle_deg: 4, pick_retries: PICK_RETRIES, rehome_every: REHOME_EVERY,
  };
  private cal = { red: [0.62, 0.20, 0.16], blue: [0.16, 0.29, 0.60], emptyClear: 250 };
  private lastGood: number[] = [0, 0, 0]; private lastColour: Colour = 'UNKNOWN';
  private chroma(v: Rgbc) { const c = v.c || 1; return [v.r / c, v.g / c, v.b / c]; }
  private decideColour(v: Rgbc): Colour {
    if (v.c < this.cal.emptyClear * this.params.empty_ratio) return 'EMPTY';
    const k = this.chroma(v), d = (a: number[], b: number[]) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    const dr = d(k, this.cal.red), db = d(k, this.cal.blue);
    if (dr < db * this.params.confident_ratio) return 'RED';
    if (db < dr * this.params.confident_ratio) return 'BLUE';
    if (this.params.use_last_trick && (this.lastColour === 'RED' || this.lastColour === 'BLUE'))
      return d(k, this.lastGood) < this.params.same_as_last ? this.lastColour : this.lastColour === 'RED' ? 'BLUE' : 'RED';
    return 'UNKNOWN';
  }
  private gripOffset(a: number) {
    const m = this.params.jiggle_mm, g = this.params.jiggle_deg;
    return [[0, 0, 0], [m, 0, g], [-m, 0, -g], [0, m, 0], [0, -m, 0]][a % 5];
  }

  // ───────────── tasks ─────────────
  private colourT0 = 0;
  private colourStart() { this.colourT0 = this.now; }
  private colourDone(): { colour: Colour; rgbc: Rgbc } | null {
    if (this.now - this.colourT0 < TIMING.colourSample * TIMING.colourSamples) return null;
    const v = this.reading();
    return { colour: this.decideColour(v), rgbc: v };
  }

  private h = { phase: 'start', i: 0, a0: 0, t0: 0, why: 'none' };
  private readonly ORDER = [1, 2, 0];
  private hgo(phase: string, j: number, dps: number) {
    this.h.phase = phase; this.h.t0 = this.now; this.h.a0 = this.q[j];
    this.mv = null; this.wp = [];
    this.run = { j, v: (dps >= 0 ? HOME_DIR[j] : -HOME_DIR[j]) * Math.abs(dps) };
  }
  private homingUpdate(): 'running' | 'done' | 'failed' {
    const h = this.h, j = this.ORDER[Math.min(h.i, 2)], travel = Math.abs(this.q[j] - h.a0);
    const fail = (why: string) => { this.stop(); h.why = why; return 'failed' as const; };
    if (!['start', 'next', 'gohome'].includes(h.phase) && this.now - h.t0 > 15000) return fail('timeout');
    switch (h.phase) {
      case 'start': this.limit(j) ? this.hgo('leave', j, -HOMING.fast) : this.hgo('fast', j, HOMING.fast); break;
      case 'leave':
        if (!this.limit(j)) this.hgo('fast', j, HOMING.fast);
        else if (travel > HOMING.leaveMax) return fail('broken_wire');
        break;
      case 'fast':
        if (this.limit(j)) this.hgo('backoff', j, -HOMING.fast);
        else if (travel > JOINT_MAX[j] - JOINT_MIN[j] + HOMING.margin) return fail('not_found');
        break;
      case 'backoff':
        if (travel >= HOMING.backoff && !this.limit(j)) this.hgo('slow', j, HOMING.slow);
        else if (travel > HOMING.leaveMax) return fail('stuck');
        break;
      case 'slow':
        if (this.limit(j)) { this.run = null; this.q[j] = HOME_SWITCH_DEG[j]; h.phase = 'next'; }
        else if (travel > HOMING.slowMax) return fail('not_found');
        break;
      case 'next':
        if (++h.i < 3) { h.phase = 'start'; break; }
        if (!this.moveTo(this.poses.HOME, MOVE.home)) return fail('home_pose');
        h.phase = 'gohome'; break;
      case 'gohome': if (!this.busy()) return 'done'; break;
    }
    return 'running';
  }

  private seq = { phase: '', grip: CELL.cube as Pose, above: CELL.cube as Pose, j6: 0, ok: true };
  private pickStart(attempt: number) {
    const [dx, dy, dr] = this.gripOffset(attempt), c = this.poses.CUBE;
    const g = { x: c.x + dx, y: c.y + dy, z: c.z, roll: c.roll + dr };
    this.seq = { phase: 'above', grip: g, above: { ...g, z: SAFE_Z }, j6: 0, ok: true };
    this.grip(GRIP_OPEN_MM);
    this.seq.ok = this.moveTo(this.seq.above, MOVE.above);
  }
  private pickUpdate(): 'running' | 'done' | 'failed' {
    const s = this.seq;
    if (!s.ok) return 'failed';
    if (this.busy()) return 'running';
    const w = this.attempt === 0 ? 0 : this.params.grip_wiggle;
    switch (s.phase) {
      case 'above': if (!this.moveLine(s.grip, LINE.pickDown)) return 'failed'; s.phase = 'down'; break;
      case 'down': this.grip(GRIP_CLOSED_MM); s.phase = 'close'; s.j6 = this.q[5];
        if (w > 0) { this.joint(5, s.j6 + w, MOVE.wiggle); s.phase = 'wiggle'; } break;
      case 'wiggle': this.joint(5, s.j6, MOVE.wiggle); s.phase = 'close'; break;
      case 'close': if (!this.moveLine(s.above, LINE.pickUp)) return 'failed'; s.phase = 'up'; break;
      case 'up': return 'done';
    }
    return 'running';
  }
  private placeStart(c: Colour) {
    const drop = c === 'RED' ? this.poses.BIN_A : this.poses.BIN_B;
    this.seq = { phase: 'transit', grip: drop, above: { ...drop, z: SAFE_Z + PLACE_ABOVE_DZ }, j6: 0, ok: true };
    this.seq.ok = this.moveTo(this.seq.above, MOVE.transit);
  }
  private placeUpdate(): 'running' | 'done' | 'failed' {
    const s = this.seq;
    if (!s.ok) return 'failed';
    if (this.busy()) return 'running';
    switch (s.phase) {
      case 'transit': if (!this.moveLine(s.grip, LINE.placeDown)) return 'failed'; s.phase = 'down'; break;
      case 'down': this.grip(GRIP_OPEN_MM, 250); s.phase = 'open'; break;
      case 'open': if (!this.moveLine(s.above, LINE.placeUp)) return 'failed'; s.phase = 'up'; break;
      case 'up': return 'done';
    }
    return 'running';
  }

  // ───────────── world physics: grasp and release ─────────────
  private slipped = false;
  private updateWorld() {
    const tip = this.where(), cube = this.world.cube, g = this.q[6];
    const near = (p: { x: number; y: number }, r: number) => Math.hypot(tip.x - p.x, tip.y - p.y) < r;
    if (cube?.where === 'cradle' && g <= GRIP_CLOSED_MM + 1 && near(CELL.cube, 15) && tip.z < CELL.cube.z + 20 && !this.slipped) {
      if (this.world.failNextPick) { this.world.failNextPick = false; this.slipped = true; }
      else cube.where = 'gripper';
    }
    if (g > GRIP_CLOSED_MM + 5) {
      this.slipped = false;
      if (cube?.where === 'gripper') {
        if (near(CELL.binA, 50)) { this.world.bins.A++; this.world.cube = null; }
        else if (near(CELL.binB, 50)) { this.world.bins.B++; this.world.cube = null; }
        else if (near(CELL.cube, 15)) cube.where = 'cradle';
        else this.world.cube = null;               // dropped on the table
      }
    }
  }

  // ───────────── state machine (port of fsm.c) ─────────────
  state: StateName = 'BOOT'; err = 0; running = false; homed = false; cycles = 0;
  private manual = false; private calTarget: Colour | null = null;
  private attempt = 0; private colourTries = 0; private cubeColour: Colour = 'UNKNOWN'; private cubeRgbc = SENSOR.EMPTY;
  private tEnter = 0; private lastReading: Colour = 'UNKNOWN';
  poses: Record<string, Pose> = { HOME: { ...CELL.home }, CUBE: { ...CELL.cube }, BIN_A: { ...CELL.binA }, BIN_B: { ...CELL.binB } };
  private pendingDone: string | null = null; private tTel = 0;

  private ev(what: string, arg?: string, n?: number) { this.out(`EV ${what}${arg ? ' ' + arg : ''}${n !== undefined ? ' ' + n : ''}`); }
  private enter(s: StateName) {
    this.state = s; this.tEnter = this.now;
    if (s === 'HOMING') { this.homed = false; this.h = { phase: 'start', i: 0, a0: 0, t0: this.now, why: 'none' }; }
    if (s === 'READ_COLOUR' || s === 'VERIFY') this.colourStart();
    if (s === 'PICK') this.pickStart(this.attempt);
    if (s === 'PLACE') this.placeStart(this.cubeColour);
    if (s === 'RETURN') this.seq.ok = this.moveTo(this.poses.HOME, MOVE.home);
    if (s === 'ERROR' || s === 'ESTOP') { this.stop(); this.running = false; this.manual = false; this.calTarget = null; }
  }
  private fail(e: number) { this.err = e; this.enter('ERROR'); this.ev('ERROR', ERRORS[e]); }
  private fsmBusy() { return this.manual || this.state === 'HOMING' || this.calTarget !== null; }

  boot() {
    this.state = 'BOOT'; this.tEnter = this.now; this.err = 0; this.running = this.homed = this.manual = false;
    this.ev('BOOT');
  }

  private fsmTick() {
    if (this.estop()) {
      if (this.state !== 'ESTOP') { this.err = 5; this.homed = false; this.enter('ESTOP'); this.ev('ESTOP'); }
      return;
    }
    if (this.manual && !this.busy()) this.manual = false;
    let r;
    switch (this.state) {
      case 'BOOT': if (this.now - this.tEnter > TIMING.boot) this.enter('HOMING'); break;
      case 'HOMING':
        r = this.homingUpdate();
        if (r === 'done') { this.homed = true; this.ev('HOMED'); this.enter('IDLE'); }
        if (r === 'failed') { this.ev('HOME_FAIL', this.h.why, this.ORDER[Math.min(this.h.i, 2)] + 1); this.fail(1); }
        break;
      case 'IDLE':
        if (this.running && !this.manual && this.ir()) { this.colourTries = 0; this.attempt = 0; this.enter('READ_COLOUR'); }
        break;
      case 'READ_COLOUR': {
        const c = this.colourDone(); if (!c) break;
        this.lastReading = c.colour;
        if (c.colour === 'RED' || c.colour === 'BLUE') { this.cubeColour = c.colour; this.cubeRgbc = c.rgbc; this.ev('COLOUR', c.colour); this.enter('PICK'); }
        else if (++this.colourTries < COLOUR_TRIES) this.colourStart();
        else this.fail(2);
        break;
      }
      case 'PICK':
        r = this.pickUpdate();
        if (r === 'done') this.enter('VERIFY');
        if (r === 'failed') this.fail(4);
        break;
      case 'VERIFY': {
        const c = this.colourDone(); if (!c) break;
        this.lastReading = c.colour;
        if (!this.ir() && c.colour === 'EMPTY') {
          this.lastColour = this.cubeColour; this.lastGood = this.chroma(this.cubeRgbc);
          this.ev('PICKED', undefined, this.attempt); this.enter('PLACE');
        } else if (++this.attempt < 1 + this.params.pick_retries) { this.ev('PICK_RETRY', undefined, this.attempt); this.enter('PICK'); }
        else this.fail(3);
        break;
      }
      case 'PLACE':
        r = this.placeUpdate();
        if (r === 'done') { this.ev('PLACED', this.cubeColour); this.enter('RETURN'); }
        if (r === 'failed') this.fail(4);
        break;
      case 'RETURN':
        if (!this.seq.ok) { this.fail(4); break; }
        if (this.busy()) break;
        this.cycles++; this.ev('CYCLE', undefined, this.cycles);
        if (this.params.rehome_every > 0 && this.cycles % this.params.rehome_every === 0) { this.ev('REHOME'); this.enter('HOMING'); }
        else this.enter('IDLE');
        break;
      case 'TEACH':
        if (this.calTarget) {
          const c = this.colourDone(); if (!c) break;
          const k = this.chroma(c.rgbc);
          if (this.calTarget === 'EMPTY') this.cal.emptyClear = c.rgbc.c;
          if (this.calTarget === 'RED') this.cal.red = k;
          if (this.calTarget === 'BLUE') this.cal.blue = k;
          this.ev('CAL', this.calTarget); this.calTarget = null;
        }
        break;
    }
  }

  // ───────────── console (port of console.c) ─────────────
  input(raw: string) {
    const tok = raw.trim().split(/\s+/).filter(Boolean);
    if (!tok.length) return;
    const cmd = tok[0].toUpperCase(), args = tok.slice(1), argc = args.length;
    const okLine = (data = '') => this.out(`OK ${cmd}${data ? ' ' + data : ''}`);
    const errLine = (why: string) => this.out(`ERR ${cmd} ${why}`);
    const COMMANDS = ['PING', 'STATUS', 'HOME', 'RUN', 'STOP', 'RESET', 'TEACH', 'DONE', 'MOVE', 'LINE', 'MOVEJ', 'JOG', 'JOGJ', 'GRIP', 'SAVE', 'GO', 'POSES', 'CAL', 'SET'];
    if (!COMMANDS.includes(cmd)) return errLine('unknown');
    const numeric = ['MOVE', 'LINE', 'MOVEJ', 'JOG', 'JOGJ', 'GRIP'].includes(cmd);
    const a = args.map(Number);
    if (numeric && a.some(v => !Number.isFinite(v))) return errLine('args');

    const canMove = (teachOnly: boolean): Why => {
      if (this.state !== 'TEACH' && (teachOnly || this.state !== 'IDLE')) return 'state';
      if (this.running) return 'running';
      if (this.fsmBusy()) return 'busy';
      return null;
    };
    const started = (ok: boolean, why: Why): Why => { if (!ok) return why; this.manual = true; return null; };
    let why: Why = null, isMove = false;
    switch (cmd) {
      case 'PING': return okLine(`ras-arm ${FW_VERSION}`);
      case 'STATUS': return okLine(`${this.state} ${ERRORS[this.err]} homed=${+this.homed} running=${+this.running}`);
      case 'POSES':
        for (const n of POSE_NAMES) { const p = this.poses[n]; this.out(`POSE ${n} ${[p.x, p.y, p.z, p.roll].map(v => v.toFixed(1)).join(' ')}`); }
        break;
      case 'HOME':
        isMove = true;
        if (!['IDLE', 'TEACH', 'ERROR'].includes(this.state)) why = 'state';
        else if (this.estop()) why = 'estop';
        else if (this.manual) why = 'busy';
        else if (this.running) why = 'running';
        else { this.err = 0; this.enter('HOMING'); }
        break;
      case 'RUN': if (this.state !== 'IDLE') why = 'state'; else if (this.manual) why = 'busy'; else this.running = true; break;
      case 'STOP': this.running = false; if (this.manual) { this.stop(); this.manual = false; } break;
      case 'TEACH': if (this.state !== 'IDLE') why = 'state'; else if (this.manual) why = 'busy'; else { this.running = false; this.enter('TEACH'); } break;
      case 'DONE': if (this.state !== 'TEACH') why = 'state'; else { this.calTarget = null; this.enter('IDLE'); } break;
      case 'RESET':
        if (this.state !== 'ERROR' && this.state !== 'ESTOP') why = 'state';
        else if (this.estop()) why = 'estop';
        else { this.err = 0; this.enter('HOMING'); }
        break;
      case 'MOVE': case 'LINE': {
        if (argc < 3 || argc > (cmd === 'MOVE' ? 5 : 4)) return errLine('args');
        const p = { x: a[0], y: a[1], z: a[2], roll: cmd === 'MOVE' && argc >= 4 ? a[3] : 0 };
        why = canMove(false) ?? started(cmd === 'MOVE' ? this.moveTo(p, argc === 5 ? a[4] : 0) : this.moveLine(p, argc === 4 ? a[3] : LINE.defaultMmS), 'unreachable');
        isMove = true; break;
      }
      case 'MOVEJ':
        if (argc < 6 || argc > 7) return errLine('args');
        why = canMove(false) ?? started(this.moveJoints(a.slice(0, 6), argc === 7 ? a[6] : 0), 'limit'); isMove = true; break;
      case 'JOG': {
        if (argc < 3 || argc > 4) return errLine('args');
        why = canMove(true);
        if (!why) { const p = this.where(); why = started(this.moveTo({ x: p.x + a[0], y: p.y + a[1], z: p.z + a[2], roll: p.roll + (a[3] ?? 0) }, MOVE.jog), 'unreachable'); }
        isMove = true; break;
      }
      case 'JOGJ': {
        if (argc !== 2 || !Number.isInteger(a[0]) || a[0] < 1 || a[0] > 7) return errLine('args');
        why = canMove(true);
        if (!why) {
          const j = a[0] - 1, t = this.targets(); t[j] += a[1];
          if (j === 6) { this.grip(t[6]); why = started(true, null); }
          else why = started(this.moveJoints(t, MOVE.jog), 'limit');
        }
        isMove = true; break;
      }
      case 'GRIP':
        if (argc !== 1) return errLine('args');
        why = canMove(false); if (!why) { this.grip(a[0]); why = started(true, null); } isMove = true; break;
      case 'SAVE': case 'GO': {
        if (argc !== 1) return errLine('args');
        const name = args[0].toUpperCase();
        if (!(POSE_NAMES as readonly string[]).includes(name)) return errLine('pose');
        why = canMove(true);
        if (!why && cmd === 'SAVE') this.poses[name] = this.where();
        if (!why && cmd === 'GO') { why = started(this.moveTo(this.poses[name], MOVE.home), 'unreachable'); isMove = true; }
        break;
      }
      case 'CAL': {
        if (argc !== 1) return errLine('args');
        const c = args[0].toUpperCase() as Colour;
        if (!['EMPTY', 'RED', 'BLUE'].includes(c)) return errLine('args');
        why = canMove(true); if (!why) { this.calTarget = c; this.colourStart(); } isMove = true; break;
      }
      case 'SET': {
        if (argc !== 2 || !Number.isFinite(Number(args[1]))) return errLine('args');
        if (this.state !== 'IDLE' && this.state !== 'TEACH') { why = 'state'; break; }
        if (!(args[0] in this.params)) return errLine('key');
        this.params[args[0]] = Number(args[1]); break;
      }
    }
    if (why) return errLine(REASON[why]);
    okLine();
    if (isMove) this.pendingDone = cmd;
  }

  private telemetry() {
    const p = this.where(), i = (v: number) => Math.round(v);
    this.out(['T', i(this.now), this.state, ...this.q.map(v => i(v * 10)), i(p.x), i(p.y), i(p.z),
      +this.ir(), COLOURS.indexOf(this.lastReading), this.err, this.limits().map(Number).join(''), +this.estop()].join(','));
  }

  /** advance simulated time */
  tick(ms: number) {
    for (let t = 0; t < ms; t += TICK_MS) {
      this.now += TICK_MS;
      if (this.world.powerOk) this.updateMotion(TICK_MS);
      this.updateSensors();
      this.updateWorld();
      this.fsmTick();
      if (this.pendingDone && !this.fsmBusy()) {
        if (this.state !== 'ERROR' && this.state !== 'ESTOP') this.out(`DONE ${this.pendingDone}`);
        this.pendingDone = null;
      }
      if (this.now - this.tTel >= TIMING.telemetry) { this.tTel = this.now; this.telemetry(); }
    }
  }

  // ───────────── virtual-mode extras (buttons in the UI) ─────────────
  placeCube(colour: CubeColour) { if (!this.world.cube) this.world.cube = { colour, where: 'cradle' }; }
  removeCube() { if (this.world.cube?.where === 'cradle') this.world.cube = null; }
  failNextPick() { this.world.failNextPick = true; }
  setPower(ok: boolean) { this.world.powerOk = ok; }
  breakSwitch(j: number, broken: boolean) { this.world.brokenSwitch[j] = broken; }
  states() { return STATES; }
}
