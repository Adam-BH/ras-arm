// One interface for the real board (SerialLink) and the in-browser firmware (VirtualLink).
// BaseLink holds what both share: line routing and the command queue
// (one command in flight, wait for its OK/ERR, 2 s timeout, then the next).
import { classify, commandName, parseReply, parseTelemetry, type Reply, type Telemetry } from './protocol';

export type LogDir = 'in' | 'out' | 'info';
export interface ArmLink {
  connect(): Promise<void>;
  disconnect(): void;
  send(cmd: string): Promise<Reply>;
  onTelemetry(cb: (t: Telemetry) => void): void;
  onEvent(cb: (e: string) => void): () => void;           // "EV …" and "DONE …" lines (prefix kept); returns unsubscribe
  onLog(cb: (dir: LogDir, line: string) => void): void;   // every raw line, for the console
  onStatus(cb: (connected: boolean, info: string) => void): void;
}

type Pending = { cmd: string; line: string; resolve: (r: Reply) => void; reject: (e: Error) => void; data: string[]; timer?: ReturnType<typeof setTimeout> };

export abstract class BaseLink implements ArmLink {
  timeoutMs = 2000;
  private tel: ((t: Telemetry) => void)[] = [];
  private ev: ((e: string) => void)[] = [];
  private log: ((d: LogDir, l: string) => void)[] = [];
  private status: ((c: boolean, i: string) => void)[] = [];
  private queue: Pending[] = [];
  private current: Pending | null = null;

  abstract connect(): Promise<void>;
  abstract disconnect(): void;
  protected abstract write(line: string): Promise<void> | void;

  onTelemetry(cb: (t: Telemetry) => void) { this.tel.push(cb); }
  onEvent(cb: (e: string) => void) { this.ev.push(cb); return () => { this.ev = this.ev.filter(f => f !== cb); }; }
  onLog(cb: (d: LogDir, l: string) => void) { this.log.push(cb); }
  onStatus(cb: (c: boolean, i: string) => void) { this.status.push(cb); }

  send(cmd: string): Promise<Reply> {
    const line = cmd.trim();
    return new Promise((resolve, reject) => {
      this.queue.push({ cmd: commandName(line), line, resolve, reject, data: [] });
      this.pump();
    });
  }

  protected emitStatus(connected: boolean, info: string) { this.status.forEach(cb => cb(connected, info)); this.info(info); }
  protected info(line: string) { this.log.forEach(cb => cb('info', line)); }

  /** clear the queue (disconnect): every waiting command fails */
  protected failAll(reason: string) {
    const all = [...(this.current ? [this.current] : []), ...this.queue];
    this.queue = []; this.current = null;
    for (const p of all) { clearTimeout(p.timer); p.reject(new Error(reason)); }
  }

  private pump() {
    if (this.current || !this.queue.length) return;
    const p = (this.current = this.queue.shift()!);
    p.timer = setTimeout(() => {
      if (this.current !== p) return;
      this.current = null; p.reject(new Error(`timeout: ${p.line}`)); this.pump();
    }, this.timeoutMs);
    this.log.forEach(cb => cb('out', p.line));
    Promise.resolve(this.write(p.line + '\n')).catch(e => {
      if (this.current === p) { clearTimeout(p.timer); this.current = null; p.reject(e); this.pump(); }
    });
  }

  /** feed one received line (without the newline) */
  protected handleLine(raw: string) {
    const line = raw.replace(/\r$/, '');
    if (!line) return;
    const kind = classify(line);
    if (kind === 'telemetry') {
      const t = parseTelemetry(line);
      if (t) this.tel.forEach(cb => cb(t));
      return;                                         // not logged: 20 lines per second
    }
    this.log.forEach(cb => cb('in', line));
    if (kind === 'event' || kind === 'done') { this.ev.forEach(cb => cb(line)); return; }
    const p = this.current;
    if (kind === 'reply') {
      const r = parseReply(line);
      if (p && r && r.cmd === p.cmd) {
        clearTimeout(p.timer); this.current = null;
        p.resolve({ ...r, data: p.data });
        this.pump();
      }
      return;
    }
    if (p) p.data.push(line);                         // e.g. POSE lines before "OK POSES"
  }
}
