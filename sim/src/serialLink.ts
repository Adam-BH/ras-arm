// sim/src/serialLink.ts — talk to the REAL arm from the browser (Chrome Web Serial).
// Same text protocol as console.c: send commands, receive "T,..." telemetry lines.

export type Telemetry = {
  ms: number; state: string;
  joints: number[];           // J1..J6 in degrees, grip in mm (index 6)
  tip: { x: number; y: number; z: number };
  ir: boolean; colour: 'EMPTY' | 'RED' | 'BLUE' | 'UNKNOWN';
};

const COLOURS = ['EMPTY', 'RED', 'BLUE', 'UNKNOWN'] as const;

export class SerialLink {
  private port?: SerialPort;
  private writer?: WritableStreamDefaultWriter<Uint8Array>;
  onTelemetry: (t: Telemetry) => void = () => {};
  onReply: (line: string) => void = () => {};

  async connect() {
    this.port = await navigator.serial.requestPort();          // user picks the Arduino
    await this.port.open({ baudRate: 115200 });
    this.writer = this.port.writable!.getWriter();
    this.readLoop();
  }

  send(cmd: string) {                                         // e.g. "JOG 0 0 -2"
    this.writer?.write(new TextEncoder().encode(cmd + '\n'));
  }

  private async readLoop() {
    const reader = this.port!.readable!.pipeThrough(new TextDecoderStream()).getReader();
    let buf = '';
    for (;;) {
      const { value, done } = await reader.read();
      if (done) break;
      buf += value;
      let i;
      while ((i = buf.indexOf('\n')) >= 0) {
        const line = buf.slice(0, i).trim(); buf = buf.slice(i + 1);
        if (line.startsWith('T,')) this.onTelemetry(parse(line)); else if (line) this.onReply(line);
      }
    }
  }
}

function parse(line: string): Telemetry {
  const f = line.split(',');                 // T,ms,state,j1..j6,grip,x,y,z,ir,col
  const n = (k: number) => Number(f[k]);
  return {
    ms: n(1), state: f[2],
    joints: [3, 4, 5, 6, 7, 8, 9].map(k => n(k) / 10),
    tip: { x: n(10), y: n(11), z: n(12) },
    ir: f[13] === '1', colour: COLOURS[n(14)] ?? 'UNKNOWN',
  };
}
