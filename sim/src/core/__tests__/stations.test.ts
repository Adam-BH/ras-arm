// stations.ts must match the float64 reference (tools/ik_reference.py) — the C stations_validate()
// is tested on the very same vectors in firmware/test, so the editor's shading ≡ the arm's answer.
import { describe, expect, it } from 'vitest';
import { cadLayout, ST_ERRORS, STATION_TYPES, StationTable, validate } from '../stations';
import vectors from './station_vectors.json';

describe('station validation vs golden vectors', () => {
  it('covers every reason', () => {
    const seen = new Set((vectors as number[][]).map(v => v[6]));
    for (const k of [0, 1, 2, 3, 4, 5]) expect(seen.has(k)).toBe(true);
  });

  it('agrees on every vector', () => {
    const cad = cadLayout().stations, bad: string[] = [];
    for (const [x, y, z, roll, dz, type, want] of vectors as number[][]) {
      const got = validate({ x, y, z, roll }, dz, STATION_TYPES[type], cad);
      if (got !== ST_ERRORS[want]) bad.push(`(${x},${y},${z},${roll}) ${STATION_TYPES[type]}: ${got} ≠ ${ST_ERRORS[want]}`);
    }
    expect(bad.slice(0, 10)).toEqual([]);
  });

  it('accepts every CAD station', () => {
    const { stations } = cadLayout();
    stations.forEach((s, i) => expect(validate(s, s.aboveDz, s.type, stations, i)).toBe('OK'));
  });
});

describe('StationTable', () => {
  it('add / set / delete / full / protected / duplicate', () => {
    const t = new StationTable();
    expect(t.add('REJECT', 'BIN', { x: 150, y: 150, z: 80, roll: 45 })).toBe('OK');
    expect(t.add('REJECT', 'BIN', { x: 150, y: -150, z: 80, roll: -45 })).toBe('NAME');
    expect(t.add('bad', 'BIN', { x: 150, y: -150, z: 80, roll: -45 })).toBe('NAME');
    expect(t.add('CUBE2', 'PICK', { x: 150, y: -150, z: 80, roll: -45 })).toBe('FULL');
    expect(t.add('NEAR', 'BIN', { x: 150, y: 160, z: 80, roll: 45 })).toBe('TOO_CLOSE');
    expect(t.set(4, { x: 600, y: 0, z: 80, roll: 0 }, 60)).toBe('UNREACHABLE');
    expect(t.layout.stations[4].x).toBe(150);
    expect(t.del(0)).toBe('PROTECTED');
    expect(t.del(1)).toBe('PROTECTED');
    expect(t.map('UNKNOWN', 'REJECT')).toBe('OK');
    expect(t.map('RED', 'HOME')).toBe('NAME');
    expect(t.del(4)).toBe('OK');
    expect(t.layout.map.UNKNOWN).toBeNull();
    for (const [i, p] of [[0, 220, 78, 90], [150, 150, 80, 45], [230, 90, 80, 21], [230, -90, 80, -21], [150, -150, 80, -45]].entries())
      expect(t.add(`B${i}`, 'BIN', { x: p[0], y: p[1], z: p[2], roll: p[3] }), `B${i}`).toBe(i < 4 ? (i === 0 ? 'TOO_CLOSE' : 'OK') : 'OK');
    expect(t.layout.stations.length).toBe(8);
    expect(t.add('MORE', 'BIN', { x: -60, y: 200, z: 80, roll: 90 })).toBe('FULL');
  });

  it('commit / revert / factory', () => {
    const t = new StationTable();
    t.add('REJECT', 'BIN', { x: 150, y: 150, z: 80, roll: 45 });
    t.revert();
    expect(t.find('REJECT')).toBe(-1);
    t.add('REJECT', 'BIN', { x: 150, y: 150, z: 80, roll: 45 });
    t.commit();
    expect(t.version).toBe(1);
    t.factory();
    expect(t.dirty).toBe(true);
    t.revert();
    expect(t.find('REJECT')).toBe(4);
  });
});
