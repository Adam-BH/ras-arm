// VirtualLink end to end: the UI's view of the virtual arm (commands, replies, events, telemetry).
import { describe, expect, it } from 'vitest';
import { VirtualLink } from '../VirtualLink';
import type { Telemetry } from '../protocol';

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
    expect(s.link.fw.world.bins).toEqual({ A: 0, B: 1 });
    await s.runUntil(() => s.tel()?.state === 'IDLE', 2000);
    expect(s.tel()?.state).toBe('IDLE');
  });

  it('teach flow: TEACH, JOG, SAVE CUBE, GO CUBE, POSES, DONE', async () => {
    const s = await setup();
    await s.runUntil(() => s.events.includes('EV HOMED'));
    expect((await s.send('JOG 0 0 -10')).text).toBe('state');     // TEACH only
    expect((await s.send('TEACH')).ok).toBe(true);
    expect((await s.send('JOG 0 0 -20')).ok).toBe(true);
    expect((await s.send('JOG 0 0 1')).text).toBe('busy');
    expect(await s.runUntil(() => s.events.includes('DONE JOG'))).toBe(true);
    expect((await s.send('SAVE CUBE')).ok).toBe(true);
    const poses = await s.send('POSES');
    expect(poses.data).toContain('POSE CUBE 160.0 0.0 150.0 0.0');
    expect((await s.send('JOGJ 1 20')).ok).toBe(true);
    await s.runUntil(() => s.events.filter(e => e === 'DONE JOGJ').length === 1);
    expect((await s.send('GO CUBE')).ok).toBe(true);
    expect(await s.runUntil(() => s.events.includes('DONE GO'))).toBe(true);
    s.link.advance(60);
    expect(s.tel()?.tip.z).toBeCloseTo(150, -1);
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

  it('times out a command with no reply', async () => {
    const link = new VirtualLink({ autoTick: false });
    link.timeoutMs = 10;
    await link.connect();
    link.fw.input = () => {};                        // a board that never answers
    await expect(link.send('PING')).rejects.toThrow('timeout');
  });
});
