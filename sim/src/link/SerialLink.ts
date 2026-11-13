// SerialLink: the REAL arm over USB with the Web Serial API (Chrome / Edge).
// 115200 baud, line splitting; the command queue lives in BaseLink.
import { BaseLink } from './ArmLink';

export class SerialLink extends BaseLink {
  private port?: SerialPort;
  private writer?: WritableStreamDefaultWriter<Uint8Array>;
  private reader?: ReadableStreamDefaultReader<string>;
  private closing = false;
  private enc = new TextEncoder();

  static supported() { return typeof navigator !== 'undefined' && 'serial' in navigator; }

  /** first time: the browser asks the user to pick the Arduino */
  async connect() {
    if (!SerialLink.supported()) throw new Error('Web Serial not available: use Chrome or Edge');
    await this.open(await navigator.serial.requestPort());
  }

  /** after an unplug: reopen a port the user already granted, no picker */
  async reconnect() {
    const [port] = await navigator.serial.getPorts();
    if (!port) return this.connect();
    await this.open(port);
  }

  private async open(port: SerialPort) {
    await port.open({ baudRate: 115200 });
    this.port = port; this.closing = false;
    this.writer = port.writable!.getWriter();
    navigator.serial.addEventListener('disconnect', this.onUnplug);
    this.emitStatus(true, 'USB port open (the UNO reboots: ~2 s)');
    void this.readLoop();
  }

  private onUnplug = (e: Event) => {
    if ((e.target as SerialPort) !== this.port) return;
    this.lost('USB cable unplugged');
  };

  private lost(why: string) {
    if (this.closing) return;
    this.closing = true;
    this.failAll(why);
    this.emitStatus(false, `${why}: press Reconnect`);
  }

  disconnect() {
    this.closing = true;
    navigator.serial?.removeEventListener('disconnect', this.onUnplug);
    this.failAll('disconnected');
    void (async () => {
      try { await this.reader?.cancel(); } catch { /* already closed */ }
      try { this.writer?.releaseLock(); } catch { /* already released */ }
      try { await this.port?.close(); } catch { /* already closed */ }
    })();
    this.emitStatus(false, 'disconnected');
  }

  protected async write(line: string) {
    if (!this.writer) throw new Error('not connected');
    await this.writer.write(this.enc.encode(line));
  }

  private async readLoop() {
    const decoder = new TextDecoderStream();
    void this.port!.readable!.pipeTo(decoder.writable as WritableStream<Uint8Array>).catch(() => {});
    this.reader = decoder.readable.getReader();
    let buf = '';
    try {
      for (;;) {
        const { value, done } = await this.reader.read();
        if (done) break;
        buf += value;
        let i;
        while ((i = buf.indexOf('\n')) >= 0) { this.handleLine(buf.slice(0, i)); buf = buf.slice(i + 1); }
        if (buf.length > 512) buf = '';                 // garbage (wrong baud?): drop it
      }
    } catch {
      this.lost('serial read error');
    } finally {
      this.reader.releaseLock();
    }
  }
}
