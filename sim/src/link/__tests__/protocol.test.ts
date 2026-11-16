import { describe, expect, it } from 'vitest';
import { classify, commandName, fmt, formatTelemetry, parseLayout, parsePose, parseReply, parseTelemetry } from '../protocol';

describe('protocol', () => {
  const line = 'T,123456,PICK,0,470,939,0,391,-40,450,220,0,24,1,1,0,010,0';

  it('parses telemetry (angles ×10, integers only)', () => {
    const t = parseTelemetry(line)!;
    expect(t.ms).toBe(123456);
    expect(t.state).toBe('PICK');
    expect(t.joints).toEqual([0, 47, 93.9, 0, 39.1, -4, 45]);
    expect(t.tip).toEqual({ x: 220, y: 0, z: 24 });
    expect(t.ir).toBe(true);
    expect(t.colour).toBe('RED');
    expect(t.errName).toBe('NONE');
    expect(t.limits).toEqual([false, true, false]);
    expect(t.estop).toBe(false);
  });

  it('round-trips telemetry', () => {
    expect(formatTelemetry(parseTelemetry(line)!)).toBe(line);
  });

  it('rejects malformed telemetry', () => {
    expect(parseTelemetry('T,1,IDLE,0')).toBeNull();
    expect(parseTelemetry(line.replace(',470,', ',47.0,'))).toBeNull();   // integers only
    expect(parseTelemetry(line.replace(',010,', ',01,'))).toBeNull();
  });

  it('parses replies', () => {
    expect(parseReply('OK PING ras-arm 1.0.0')).toEqual({ ok: true, cmd: 'PING', text: 'ras-arm 1.0.0' });
    expect(parseReply('ERR MOVE unreachable')).toEqual({ ok: false, cmd: 'MOVE', text: 'unreachable' });
    expect(parseReply('OK RUN')).toEqual({ ok: true, cmd: 'RUN', text: '' });
    expect(parseReply('EV HOMED')).toBeNull();
  });

  it('classifies lines', () => {
    expect(classify(line)).toBe('telemetry');
    expect(classify('OK RUN')).toBe('reply');
    expect(classify('DONE MOVE')).toBe('done');
    expect(classify('EV PICK_RETRY 1')).toBe('event');
    expect(classify('POSE HOME 160.0 0.0 170.0 0.0')).toBe('data');
  });

  it('parses poses and formats commands', () => {
    expect(parsePose('POSE BIN_B 0.0 -220.0 78.0 0.0')).toEqual({ name: 'BIN_B', x: 0, y: -220, z: 78, roll: 0 });
    expect(commandName('  move 1 2 3')).toBe('MOVE');
    expect(fmt.move(200, 0, 100.04)).toBe('MOVE 200 0 100 0');
    expect(fmt.movej([0, 30, 90, 0, 60, 0, 45], 800)).toBe('MOVEJ 0 30 90 0 60 0 800');
    expect(fmt.jog(0, 0, -0.2)).toBe('JOG 0 0 -0.2 0');
  });

  it('telemetry with the layout version (field 19) round-trips; 18 fields still parse', () => {
    const t = parseTelemetry(line + ',7')!;
    expect(t.layout).toBe(7);
    expect(formatTelemetry(t)).toBe(line + ',7');
    expect(parseTelemetry(line)!.layout).toBeUndefined();
    expect(parseTelemetry(line + ',x')).toBeNull();
  });

  it('parses ST LIST and formats ST SET / ST MAP', () => {
    const l = parseLayout(['ST,0,HOME,PARK,160,0,170,0,60', 'ST,4,REJECT,BIN,150,150,80,45,50', 'garbage', 'MAP,RED,BIN_A', 'MAP,UNKNOWN,-']);
    expect(l.stations).toEqual([
      { name: 'HOME', type: 'PARK', x: 160, y: 0, z: 170, roll: 0, aboveDz: 60 },
      { name: 'REJECT', type: 'BIN', x: 150, y: 150, z: 80, roll: 45, aboveDz: 50 },
    ]);
    expect(l.map).toEqual({ RED: 'BIN_A', BLUE: null, UNKNOWN: null });
    expect(fmt.stSet(l.stations[1])).toBe('ST SET REJECT 150 150 80 45 50 BIN');
    expect(fmt.stMap('UNKNOWN', null)).toBe('ST MAP UNKNOWN -');
  });
});
