// One store for the whole UI: connection, telemetry + history, poses, events, serial log.
import { create } from 'zustand';
import type { ArmLink, LogDir } from '../link/ArmLink';
import { SerialLink } from '../link/SerialLink';
import { VirtualLink } from '../link/VirtualLink';
import { parsePose, type Colour, type Reply, type Telemetry } from '../link/protocol';
import type { Pose } from '../core/kinematics';

export type Mode = 'virtual' | 'usb';
export type LogLine = { id: number; dir: LogDir; text: string; t: number };
export type Sample = { t: number; j: number[] };

const HISTORY_MS = 20000, LOG_MAX = 400;
let logId = 0;

type ArmState = {
  mode: Mode; link: ArmLink | null; connected: boolean; status: string; version: string;
  tel: Telemetry | null; history: Sample[]; log: LogLine[];
  poses: Record<string, Pose>; cycles: number; cubeColour: Colour; lastEvent: string;
  setMode(m: Mode): void;
  connect(): Promise<void>;
  reconnect(): Promise<void>;
  disconnect(): void;
  send(cmd: string): Promise<Reply | null>;
  refreshPoses(): Promise<void>;
};

export const useArm = create<ArmState>((set, get) => {
  const addLog = (dir: LogDir, text: string) =>
    set(s => ({ log: [...s.log.slice(-(LOG_MAX - 1)), { id: logId++, dir, text, t: Date.now() }] }));

  const wire = (link: ArmLink) => {
    link.onLog(addLog);
    link.onStatus((connected, info) => set({ connected, status: info }));
    link.onTelemetry(t => set(s => {
      const now = performance.now();
      const history = [...s.history, { t: now, j: t.joints }];
      while (history.length && now - history[0].t > HISTORY_MS) history.shift();
      return { tel: t, history };
    }));
    link.onEvent(e => {
      const [, what, arg] = e.split(' ');
      if (what === 'CYCLE') set({ cycles: Number(arg) });
      if (what === 'COLOUR') set({ cubeColour: arg as Colour });
      if (what === 'HOMED') void get().refreshPoses();
      if (e.startsWith('DONE SAVE') || e === 'DONE GO') void get().refreshPoses();
      set({ lastEvent: e });
    });
  };

  // the UNO reboots when the port opens: keep pinging until the firmware answers
  const hello = async () => {
    for (let i = 0; i < 4 && get().connected; i++) {
      const r = await get().send('PING');
      if (r?.ok) { set({ version: r.text.replace('ras-arm ', '') }); await get().refreshPoses(); return; }
    }
  };

  return {
    mode: 'virtual', link: null, connected: false, status: 'not connected', version: '',
    tel: null, history: [], log: [], poses: {}, cycles: 0, cubeColour: 'UNKNOWN', lastEvent: '',

    setMode(m) { if (m !== get().mode) { get().disconnect(); set({ mode: m }); } },

    async connect() {
      get().disconnect();
      const link = get().mode === 'virtual' ? new VirtualLink() : new SerialLink();
      wire(link);
      set({ link, tel: null, history: [], cycles: 0, version: '', poses: {} });
      try { await link.connect(); await hello(); }
      catch (e) { set({ link: null, connected: false, status: (e as Error).message }); }
    },

    async reconnect() {
      const link = get().link;
      if (link instanceof SerialLink) {
        try { await link.reconnect(); await hello(); } catch (e) { set({ status: (e as Error).message }); }
      } else await get().connect();
    },

    disconnect() { get().link?.disconnect(); set({ link: null, connected: false }); },

    async send(cmd) {
      const link = get().link;
      if (!link || !get().connected) { addLog('info', `not connected: ${cmd}`); return null; }
      try { return await link.send(cmd); }
      catch (e) { addLog('info', (e as Error).message); return null; }
    },

    async refreshPoses() {
      const r = await get().send('POSES');
      if (!r?.ok) return;
      const poses: Record<string, Pose> = {};
      for (const l of r.data) { const p = parsePose(l); if (p) poses[p.name] = { x: p.x, y: p.y, z: p.z, roll: p.roll }; }
      set({ poses });
    },
  };
});

/** the virtual firmware, when in virtual mode (for the "virtual extras" and the 3D world) */
export const useVirtualFw = () => {
  const link = useArm(s => s.link);
  return link instanceof VirtualLink ? link.fw : null;
};
