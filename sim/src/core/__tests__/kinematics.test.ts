// The TS IK must match the float64 reference (tools/ik_reference.py) — the C IK is tested
// against the very same vectors in firmware/test, so TS ≡ C within 0.1 mm.
import { describe, expect, it } from 'vitest';
import { fk, ik, IK_STATUS_CODE } from '../kinematics';
import vectors from './ik_vectors.json';

type V = { pose: number[]; status: number; joints: number[]; tip: number[] | null };

describe('kinematics vs golden vectors', () => {
  it('has a meaningful set of vectors', () => {
    expect((vectors as V[]).length).toBeGreaterThan(200);
    expect((vectors as V[]).filter(v => v.status === 0).length).toBeGreaterThan(100);
  });

  it.each((vectors as V[]).map((v, i) => [i, v] as const))('vector %i', (_i, v) => {
    const [x, y, z, roll] = v.pose;
    const r = ik({ x, y, z, roll });
    expect(IK_STATUS_CODE[r.status]).toBe(v.status);
    if (v.status === 1) return;
    v.joints.forEach((q, j) => expect(Math.abs(r.q[j] - q)).toBeLessThan(0.05));
    if (v.status !== 0) return;
    const back = fk(r.q);
    expect(Math.abs(back.x - x)).toBeLessThan(0.1);
    expect(Math.abs(back.y - y)).toBeLessThan(0.1);
    expect(Math.abs(back.z - z)).toBeLessThan(0.1);
  });

  it('keeps the gripper vertical', () => {
    const r = ik({ x: 200, y: 40, z: 60, roll: 0 });
    expect(r.q[1] + r.q[2] + r.q[4]).toBeCloseTo(180, 6);
  });
});
