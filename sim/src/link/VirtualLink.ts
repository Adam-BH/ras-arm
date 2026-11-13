// VirtualLink: the same ArmLink interface, but the "board" is the TypeScript firmware twin.
import { BaseLink } from './ArmLink';
import { VirtualFirmware } from '../core/virtualFirmware';

export class VirtualLink extends BaseLink {
  fw: VirtualFirmware;
  private timer?: ReturnType<typeof setInterval>;
  private connected = false;

  /** autoTick=false: tests drive time with advance() */
  constructor(private opts: { autoTick?: boolean; periodMs?: number } = {}) {
    super();
    this.fw = new VirtualFirmware(line => { if (this.connected) this.handleLine(line); });
  }

  async connect() {
    this.connected = true;                            // one VirtualLink = one boot (the store makes a new one per Connect)
    this.fw.boot();                                   // now connected: the "EV BOOT" line is heard
    if (this.opts.autoTick !== false) {
      const p = this.opts.periodMs ?? 20;
      this.timer = setInterval(() => this.fw.tick(p), p);
    }
    this.emitStatus(true, 'virtual arm booted');
  }

  disconnect() {
    clearInterval(this.timer);
    this.connected = false;
    this.failAll('disconnected');
    this.emitStatus(false, 'virtual arm off');
  }

  advance(ms: number) { this.fw.tick(ms); }

  protected write(line: string) {
    if (!this.connected) throw new Error('not connected');
    queueMicrotask(() => this.fw.input(line));      // like a real wire: the reply comes later
  }
}
