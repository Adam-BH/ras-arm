// One store for the whole UI: connection, telemetry + history, poses, events, serial log.
import { create } from 'zustand';
import type { ArmLink, LogDir } from '../link/ArmLink';
import { SerialLink } from '../link/SerialLink';
import { VirtualLink } from '../link/VirtualLink';
import { fmt, parseLayout, type Colour, type Reply, type Telemetry } from '../link/protocol';
import { cadLayout, MAP_COLOURS, type Layout } from '../core/stations';

export type Mode = 'virtual' | 'usb';
export type LogLine = { id: number; dir: LogDir; text: string; t: number };
export type Sample = { t: number; j: number[] };

const HISTORY_MS = 20000, LOG_MAX = 400;
let logId = 0;

type ArmState = {
  mode: Mode; link: ArmLink | null; connected: boolean; status: string; version: string;
  tel: Telemetry | null; history: Sample[]; log: LogLine[];
  cycles: number; cubeColour: Colour; lastEvent: string;
  // workcell layout: mirrors the arm (ST LIST) until edited here; 'edited' = local changes not sent yet
  layout: Layout; edited: boolean; stErrors: Record<string, string>; selected: string | null; layoutVersion: number;
  setMode(m: Mode): void;
  connect(): Promise<void>;
  reconnect(): Promise<void>;
  disconnect(): void;
  send(cmd: string): Promise<Reply | null>;
  loadLayout(): Promise<void>;
  sendLayout(): Promise<string>;
  editLayout(f: (l: Layout) => Layout): void;
  select(name: string | null): void;
};

export const useArm = create<ArmState>((set, get) => {
  const addLog = (dir: LogDir, text: string) =>
    set(s => ({ log: [...s.log.slice(-(LOG_MAX - 1)), { id: logId++, dir, text, t: Date.now() }] }));

  const wire = (link: ArmLink) => {
    link.onLog(addLog);
    link.onStatus((connected, info) => set({ connected, status: info }));
    link.onTelemetry(t => set(s => {
      if (t.layout !== undefined && t.layout !== s.layoutVersion) {   // committed on the arm: re-read (unless editing)
        if (!s.edited) queueMicrotask(() => void get().loadLayout());
        s = { ...s, layoutVersion: t.layout };
      }
      const now = performance.now();
      const history = [...s.history, { t: now, j: t.joints }];
      while (history.length && now - history[0].t > HISTORY_MS) history.shift();
      return { tel: t, history, layoutVersion: s.layoutVersion };
    }));
    link.onEvent(e => {
      const [, what, arg] = e.split(' ');
      if (what === 'CYCLE') set({ cycles: Number(arg) });
      if (what === 'COLOUR') set({ cubeColour: arg as Colour });

      set({ lastEvent: e });
    });
  };

  // the UNO reboots when the port opens: keep pinging until the firmware answers
  const hello = async () => {
    for (let i = 0; i < 4 && get().connected; i++) {
      const r = await get().send('PING');
      if (r?.ok) { set({ version: r.text.replace('ras-arm ', '') }); await get().loadLayout(); return; }
    }
  };

  return {
    mode: 'virtual', link: null, connected: false, status: 'not connected', version: '',
    tel: null, history: [], log: [], cycles: 0, cubeColour: 'UNKNOWN', lastEvent: '',
    layout: cadLayout(), edited: false, stErrors: {}, selected: null, layoutVersion: -1,

    setMode(m) { if (m !== get().mode) { get().disconnect(); set({ mode: m }); } },

    async connect() {
      get().disconnect();
      const link = get().mode === 'virtual' ? new VirtualLink() : new SerialLink();
      wire(link);
      set({ link, tel: null, history: [], cycles: 0, version: '', layoutVersion: -1 });
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

    async loadLayout() {
      const r = await get().send('ST LIST');
      if (r?.ok) set({ layout: parseLayout(r.data), edited: false });
    },

    /** draft → arm: TEACH, delete extras, ST SET each, ST MAP each, ST COMMIT, re-read. The arm has the last word. */
    async sendLayout() {
      const { send, layout } = get();
      if (get().tel?.state !== 'TEACH' && !(await send('TEACH'))?.ok) return 'enter TEACH first (arm busy or not idle)';
      const arm = await send('ST LIST');
      if (!arm?.ok) return 'no answer to ST LIST';
      const errors: Record<string, string> = {};
      for (const s of parseLayout(arm.data).stations)
        if (!layout.stations.some(d => d.name === s.name)) { const r = await send(`ST DEL ${s.name}`); if (!r?.ok) errors[s.name] = r?.text ?? 'no reply'; }
      for (const s of layout.stations) { const r = await send(fmt.stSet(s)); if (!r?.ok) errors[s.name] = r?.text ?? 'no reply'; }
      for (const c of MAP_COLOURS) { const r = await send(fmt.stMap(c, layout.map[c])); if (!r?.ok) errors[`map ${c}`] = r?.text ?? 'no reply'; }
      await send('ST COMMIT');
      await get().loadLayout();                       // show what the arm actually kept
      set({ stErrors: errors });
      const n = Object.keys(errors).length;
      return n ? `sent; arm refused ${n}: ${Object.entries(errors).map(([k, v]) => `${k} ${v}`).join(', ')}` : 'sent and committed';
    },

    editLayout(f) { set(s => ({ layout: f(structuredClone(s.layout)), edited: true, stErrors: {} })); },
    select(name) { set({ selected: name }); },
  };
});

/** the virtual firmware, when in virtual mode (for the "virtual extras" and the 3D world) */
export const useVirtualFw = () => {
  const link = useArm(s => s.link);
  return link instanceof VirtualLink ? link.fw : null;
};
