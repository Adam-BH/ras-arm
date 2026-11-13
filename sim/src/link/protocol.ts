// The serial protocol (docs/serial_protocol.md): types, parsing, formatting.
export const STATES = ['BOOT', 'HOMING', 'IDLE', 'READ_COLOUR', 'PICK', 'VERIFY',
  'PLACE', 'RETURN', 'TEACH', 'ERROR', 'ESTOP'] as const;
export type StateName = (typeof STATES)[number];
export const ERRORS = ['NONE', 'HOME_FAILED', 'COLOUR_UNKNOWN', 'PICK_FAILED', 'UNREACHABLE', 'ESTOP'] as const;
export const COLOURS = ['EMPTY', 'RED', 'BLUE', 'UNKNOWN'] as const;
export type Colour = (typeof COLOURS)[number];
export const POSE_NAMES = ['HOME', 'CUBE', 'BIN_A', 'BIN_B'] as const;
export type PoseName = (typeof POSE_NAMES)[number];

export type Telemetry = {
  ms: number; state: string;
  joints: number[];              // J1..J6 deg, [6] = grip mm
  tip: { x: number; y: number; z: number };
  ir: boolean; colour: Colour; err: number; errName: string;
  limits: [boolean, boolean, boolean]; estop: boolean;
};

export type Reply = { ok: boolean; cmd: string; text: string; data: string[] };

/** T,<ms>,<state>,<j1..j6 ×10>,<grip ×10>,<x>,<y>,<z>,<ir>,<colour>,<err>,<lim1><lim2><lim3>,<estop> */
export function parseTelemetry(line: string): Telemetry | null {
  const f = line.trim().split(',');
  if (f[0] !== 'T' || f.length !== 18) return null;
  const n = (k: number) => Number(f[k]);
  for (const k of [1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 17]) if (!/^-?\d+$/.test(f[k])) return null;
  if (!/^[01]{3}$/.test(f[16])) return null;
  return {
    ms: n(1), state: f[2],
    joints: [3, 4, 5, 6, 7, 8, 9].map(k => n(k) / 10),
    tip: { x: n(10), y: n(11), z: n(12) },
    ir: f[13] === '1', colour: COLOURS[n(14)] ?? 'UNKNOWN',
    err: n(15), errName: ERRORS[n(15)] ?? `E${n(15)}`,
    limits: [f[16][0] === '1', f[16][1] === '1', f[16][2] === '1'],
    estop: f[17] === '1',
  };
}

export function formatTelemetry(t: Telemetry): string {
  const i = (v: number) => String(Math.round(v));
  return ['T', t.ms, t.state, ...t.joints.map(v => i(v * 10)), i(t.tip.x), i(t.tip.y), i(t.tip.z),
    t.ir ? 1 : 0, COLOURS.indexOf(t.colour), t.err, t.limits.map(b => (b ? 1 : 0)).join(''), t.estop ? 1 : 0].join(',');
}

export type LineKind = 'telemetry' | 'reply' | 'done' | 'event' | 'data';
export function classify(line: string): LineKind {
  if (line.startsWith('T,')) return 'telemetry';
  if (line.startsWith('OK ') || line.startsWith('ERR ')) return 'reply';
  if (line.startsWith('DONE ')) return 'done';
  if (line.startsWith('EV ')) return 'event';
  return 'data';
}

/** "OK PING ras-arm 1.0.0" → {ok, cmd: 'PING', text: 'ras-arm 1.0.0'} */
export function parseReply(line: string): Omit<Reply, 'data'> | null {
  const m = /^(OK|ERR) (\S+)(?: (.*))?$/.exec(line.trim());
  return m ? { ok: m[1] === 'OK', cmd: m[2], text: m[3] ?? '' } : null;
}

export const commandName = (cmd: string) => (cmd.trim().split(/\s+/)[0] ?? '').toUpperCase();

/** "POSE CUBE 220.0 0.0 24.0 0.0" → {name, x, y, z, roll} */
export function parsePose(line: string) {
  const m = /^POSE (\S+) (-?[\d.]+) (-?[\d.]+) (-?[\d.]+) (-?[\d.]+)$/.exec(line.trim());
  return m ? { name: m[1], x: +m[2], y: +m[3], z: +m[4], roll: +m[5] } : null;
}

const num = (v: number) => String(Math.round(v * 10) / 10);
export const fmt = {
  move: (x: number, y: number, z: number, roll = 0, ms?: number) => `MOVE ${[x, y, z, roll].map(num).join(' ')}${ms ? ` ${Math.round(ms)}` : ''}`,
  line: (x: number, y: number, z: number, mmS?: number) => `LINE ${[x, y, z].map(num).join(' ')}${mmS ? ` ${num(mmS)}` : ''}`,
  movej: (q: number[], ms?: number) => `MOVEJ ${q.slice(0, 6).map(num).join(' ')}${ms ? ` ${Math.round(ms)}` : ''}`,
  jog: (dx: number, dy: number, dz: number, dr = 0) => `JOG ${[dx, dy, dz, dr].map(num).join(' ')}`,
  jogj: (j: number, d: number) => `JOGJ ${j} ${num(d)}`,
  grip: (mm: number) => `GRIP ${num(mm)}`,
};
