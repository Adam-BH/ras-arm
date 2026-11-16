// VirtualLink end to end: the UI's view of the virtual arm (commands, replies, events, telemetry).
import { describe, expect, it } from 'vitest';
import { VirtualLink } from '../VirtualLink';
import { parseLayout, type Telemetry } from '../protocol';

async function setup() {
  const link = new VirtualLink({ autoTick: false });
  const events: string[] = []; let tel: Telemetry | null = null;
  link.onEvent(e => events.push(e));
  link.onTelemetry(t => { tel = t; });
  await link.connect();
  const send = async (cmd: string) => { const p = link.send(cmd); await Promise.resolve(); await Promise.resolve(); return p; };
  const runUntil = async (pred: () => boolean, maxMs = 60000) => {
    for (let t = 0; t < maxMs && !pred(); t += 20) { link.advance(20); await Promise.resolve(); }
    return pred();
  };
  return { link, events, tel: () => tel as Telemetry | null, send, runUntil };
}

describe('VirtualLink', () => {
  it('boots, homes, answers PING/STATUS', async () => {
    const s = await setup();
    expect(await s.runUntil(() => s.events.includes('EV HOMED'))).toBe(true);
    expect((await s.send('PING')).text).toBe('ras-arm 1.0.0');
    const st = await s.send('STATUS');
    expect(st.ok).toBe(true);
    expect(st.text).toBe('IDLE NONE homed=1 running=0');
    s.link.advance(100);
    expect(s.tel()?.state).toBe('IDLE');
    expect(s.tel()?.tip.x).toBeCloseTo(160, -1);
  });

  it('full sorting cycle with one failed pick', async () => {
    const s = await setup();
    await s.runUntil(() => s.events.includes('EV HOMED'));
    s.link.fw.placeCube('BLUE');
    s.link.fw.failNextPick();
    expect((await s.send('RUN')).ok).toBe(true);
    expect(await s.runUntil(() => s.events.includes('EV CYCLE 1'))).toBe(true);
    expect(s.events).toContain('EV COLOUR BLUE');
    expect(s.events).toContain('EV PICK_RETRY 1');
    expect(s.events).toContain('EV PLACED BLUE');
    expect(s.link.fw.world.bins).toEqual({ BIN_B: 1 });
    await s.runUntil(() => s.tel()?.state === 'IDLE', 2000);
    expect(s.tel()?.state).toBe('IDLE');
  });

  it('teach flow: TEACH, JOG, SAVE CUBE, GO CUBE, POSES, DONE', async () => {
    const s = await setup();
    await s.runUntil(() => s.events.includes('EV HOMED'));
    expect((await s.send('JOG 0 0 -10')).text).toBe('state');     // TEACH only
    expect((await s.send('MOVE 200 0 70')).ok).toBe(true);          // near the table: a pose CUBE can really have
    expect(await s.runUntil(() => s.events.includes('DONE MOVE'))).toBe(true);
    expect((await s.send('TEACH')).ok).toBe(true);
    expect((await s.send('JOG 0 0 -20')).ok).toBe(true);
    expect((await s.send('JOG 0 0 1')).text).toBe('busy');
    expect(await s.runUntil(() => s.events.includes('DONE JOG'))).toBe(true);
    expect((await s.send('SAVE CUBE')).ok).toBe(true);
    const poses = await s.send('POSES');
    expect(poses.data).toContain('POSE CUBE 200.0 0.0 50.0 0.0');
    expect((await s.send('JOGJ 1 20')).ok).toBe(true);
    await s.runUntil(() => s.events.filter(e => e === 'DONE JOGJ').length === 1);
    expect((await s.send('GO CUBE')).ok).toBe(true);
    expect(await s.runUntil(() => s.events.includes('DONE GO'))).toBe(true);
    s.link.advance(60);
    expect(s.tel()?.tip.z).toBeCloseTo(50, -1);
    expect((await s.send('DONE')).ok).toBe(true);
  });

  it('move-to: reachable and unreachable', async () => {
    const s = await setup();
    await s.runUntil(() => s.events.includes('EV HOMED'));
    expect((await s.send('MOVE 200 50 80')).ok).toBe(true);
    expect(await s.runUntil(() => s.events.includes('DONE MOVE'))).toBe(true);
    s.link.advance(60);
    expect(s.tel()?.tip).toEqual({ x: 200, y: 50, z: 80 });   // telemetry: integer mm, rounded
    const r = await s.send('MOVE 600 0 0');
    expect(r).toMatchObject({ ok: false, cmd: 'MOVE', text: 'unreachable' });
    expect((await s.send('MOVEJ 0 30 5 0 60 0')).text).toBe('limit');
  });

  it('E-stop during a pick → ESTOP → RESET → HOMING → IDLE', async () => {
    const s = await setup();
    await s.runUntil(() => s.events.includes('EV HOMED'));
    s.link.fw.placeCube('RED');
    await s.send('RUN');
    await s.runUntil(() => s.tel()?.state === 'PICK');
    s.link.fw.setPower(false);
    expect(await s.runUntil(() => s.tel()?.state === 'ESTOP', 500)).toBe(true);
    expect((await s.send('RESET')).text).toBe('estop');
    s.link.fw.setPower(true);
    expect((await s.send('RESET')).ok).toBe(true);
    expect(await s.runUntil(() => s.events.filter(e => e === 'EV HOMED').length === 2)).toBe(true);
  });

  it('broken limit-switch wire → homing fails safely', async () => {
    const s = await setup();
    s.link.fw.breakSwitch(1, true);
    expect(await s.runUntil(() => s.events.includes('EV HOME_FAIL broken_wire 2'))).toBe(true);
    s.link.advance(60);
    expect(s.tel()?.state).toBe('ERROR');
    expect(s.tel()?.errName).toBe('HOME_FAILED');
  });

  it('workcell: ST LIST / SET / MAP / GO / COMMIT, refusals, layout version in telemetry', async () => {
    const s = await setup();
    await s.runUntil(() => s.events.includes('EV HOMED'));
    const list = await s.send('ST LIST');
    expect(list.data).toEqual(['ST,0,HOME,PARK,160,0,170,0,60', 'ST,1,CUBE,PICK,220,0,24,0,60', 'ST,2,BIN_A,BIN,0,220,78,90,60',
      'ST,3,BIN_B,BIN,0,-220,78,-90,60', 'MAP,RED,BIN_A', 'MAP,BLUE,BIN_B', 'MAP,UNKNOWN,-']);
    expect((await s.send('ST SET REJECT 150 150 80 45')).text).toBe('NOT_TEACH');
    await s.send('TEACH');
    expect((await s.send('ST SET REJECT 150 150 80 45')).ok).toBe(true);
    expect((await s.send('ST SET FAR 600 0 80 0')).text).toBe('UNREACHABLE');
    expect((await s.send('ST DEL HOME')).text).toBe('PROTECTED');
    expect((await s.send('st map unknown reject')).ok).toBe(true);
    expect((await s.send('ST GO REJECT')).ok).toBe(true);
    expect(await s.runUntil(() => s.events.includes('DONE ST'))).toBe(true);
    s.link.advance(60);
    expect(s.tel()?.tip).toEqual({ x: 150, y: 150, z: 80 });
    expect((await s.send('ST ADD HERE BIN')).text).toBe('TOO_CLOSE');
    expect((await s.send('DONE')).ok).toBe(true);
    expect(s.events).toContain('EV WARN uncommitted_layout');
    await s.send('TEACH');
    expect(s.tel()?.layout).toBe(0);
    expect((await s.send('ST COMMIT')).ok).toBe(true);
    s.link.advance(60);
    expect(s.tel()?.layout).toBe(1);
    const back = parseLayout((await s.send('ST LIST')).data);
    expect(back.stations.map(st => st.name)).toEqual(['HOME', 'CUBE', 'BIN_A', 'BIN_B', 'REJECT']);
    expect(back.map).toEqual({ RED: 'BIN_A', BLUE: 'BIN_B', UNKNOWN: 'REJECT' });
  });

  it('swapped bins: RED → BIN_B', async () => {
    const s = await setup();
    await s.runUntil(() => s.events.includes('EV HOMED'));
    await s.send('TEACH'); await s.send('ST MAP RED BIN_B'); await s.send('ST COMMIT'); await s.send('DONE');
    s.link.fw.placeCube('RED');
    await s.send('RUN');
    expect(await s.runUntil(() => s.events.includes('EV CYCLE 1'))).toBe(true);
    expect(s.link.fw.world.bins).toEqual({ BIN_B: 1 });
  });

  it('reject bin: an unknown colour goes to REJECT; without one it is an ERROR', async () => {
    const s = await setup();
    await s.runUntil(() => s.events.includes('EV HOMED'));
    await s.send('TEACH');
    expect((await s.send('ST SET REJECT 150 150 80 45')).ok).toBe(true);
    expect((await s.send('ST MAP UNKNOWN REJECT')).ok).toBe(true);
    await s.send('ST COMMIT'); await s.send('DONE');
    s.link.fw.placeCube('PURPLE');
    await s.send('RUN');
    expect(await s.runUntil(() => s.events.includes('EV CYCLE 1'))).toBe(true);
    expect(s.events).toContain('EV COLOUR UNKNOWN');
    expect(s.events).toContain('EV PLACED UNKNOWN');
    expect(s.link.fw.world.bins).toEqual({ REJECT: 1 });
    await s.send('STOP');
    await s.runUntil(() => s.tel()?.state === 'IDLE', 2000);
    await s.send('TEACH'); await s.send('ST MAP UNKNOWN -'); await s.send('DONE');
    s.link.fw.placeCube('PURPLE');
    await s.send('RUN');
    expect(await s.runUntil(() => s.events.includes('EV ERROR COLOUR_UNKNOWN'))).toBe(true);
  });

  it('times out a command with no reply', async () => {
    const link = new VirtualLink({ autoTick: false });
    link.timeoutMs = 10;
    await link.connect();
    link.fw.input = () => {};                        // a board that never answers
    await expect(link.send('PING')).rejects.toThrow('timeout');
  });
});
