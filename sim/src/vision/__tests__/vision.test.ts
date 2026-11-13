// Vision pipeline: camera geometry, colour classes, detection accuracy on noisy synthetic frames, layouts.
import { describe, expect, it } from 'vitest';
import { CELL } from '../../core/config';
import { ik } from '../../core/kinematics';
import { CAMERA, CUBE_PLANE_Z, ZONE } from '../config';
import { detect, detectCubes, pixelClass, rgbToHsv, wrap45, type CubeColour } from '../detect';
import { applyH, cameraHomography, invertH, project, solveHomography, type Vec2 } from '../geometry';
import { graspRoll, rng32, scatterLayout } from '../plan';
import { cubeBox, SHADE, synthFrame } from '../synthCamera';

const H = cameraHomography(CAMERA, CUBE_PLANE_Z);

describe('camera geometry', () => {
  it('pinhole: image up = robot +x, image right = robot −y, centre under the camera', () => {
    const c = project(CAMERA, { x: CAMERA.x, y: CAMERA.y, z: 0 });
    expect(c[0]).toBeCloseTo(CAMERA.width / 2, 9);
    expect(c[1]).toBeCloseTo(CAMERA.height / 2, 9);
    expect(project(CAMERA, { x: CAMERA.x + 50, y: 0, z: 0 })[1]).toBeLessThan(c[1]);
    expect(project(CAMERA, { x: CAMERA.x, y: 50, z: 0 })[0]).toBeLessThan(c[0]);
  });

  it('homography from the pinhole model maps pixels back to the table plane', () => {
    const rand = rng32(1);
    for (let k = 0; k < 100; k++) {
      const p = { x: ZONE.xMin + rand() * 170, y: ZONE.yMin + rand() * 320, z: CUBE_PLANE_Z };
      const [x, y] = applyH(H, project(CAMERA, p));
      expect(x).toBeCloseTo(p.x, 6);
      expect(y).toBeCloseTo(p.y, 6);
      const [u, v] = applyH(invertH(H), [p.x, p.y]);
      const q = project(CAMERA, p);
      expect(u).toBeCloseTo(q[0], 6);
      expect(v).toBeCloseTo(q[1], 6);
    }
  });

  it('least squares over more than 4 noisy clicks (webcam calibration)', () => {
    const rand = rng32(2), table: Vec2[] = [], px: Vec2[] = [];
    for (let k = 0; k < 9; k++) {
      const t: Vec2 = [140 + (k % 3) * 60, -120 + Math.floor(k / 3) * 120];
      const [u, v] = project(CAMERA, { x: t[0], y: t[1], z: 0 });
      table.push(t); px.push([u + rand() - 0.5, v + rand() - 0.5]);   // ±0.5 px click error
    }
    const Hc = solveHomography(px, table), [x, y] = applyH(Hc, project(CAMERA, { x: 205, y: 30, z: 0 }));
    expect(Math.hypot(x - 205, y - 30)).toBeLessThan(1);
    expect(() => solveHomography(px.slice(0, 3), table.slice(0, 3))).toThrow();
  });
});

describe('colour classes', () => {
  it('HSV of the primaries', () => {
    expect(rgbToHsv(255, 0, 0)).toEqual([0, 1, 1]);
    expect(rgbToHsv(0, 0, 255)).toEqual([240, 1, 1]);
    expect(rgbToHsv(128, 128, 128)[1]).toBe(0);
  });

  it('cubes are red / blue; table, arm, sensor pad, grid, shadows are not', () => {
    for (const c of ['RED', 'BLUE'] as const)
      for (const rgb of [SHADE[c].top, SHADE[c].side]) expect(pixelClass(...rgb)).toBe(c === 'RED' ? 1 : 2);
    const others: [number, number, number][] = [[42, 47, 56], [227, 163, 59], [155, 89, 182], [61, 69, 82], [91, 101, 119], [201, 206, 214], [30, 8, 8]];
    for (const rgb of others) expect(pixelClass(...rgb)).toBe(0);
  });
});

describe('detection', () => {
  it('finds every cube with its colour, centre and yaw (200 cubes, noisy frames)', () => {
    const rand = rng32(42), colours: CubeColour[] = ['RED', 'BLUE', 'RED', 'BLUE'];
    let maxPos = 0, maxYaw = 0, n = 0;
    for (let k = 0; k < 50; k++) {
      const cubes = scatterLayout(colours, rand);
      const found = detectCubes(synthFrame(CAMERA, cubes.map(cubeBox), 8, rand), H);
      expect(found).toHaveLength(cubes.length);
      for (const c of cubes) {
        const d = found.find(f => Math.hypot(f.x - c.x, f.y - c.y) < 10);
        expect(d?.colour).toBe(c.colour);
        maxPos = Math.max(maxPos, Math.hypot(d!.x - c.x, d!.y - c.y));
        maxYaw = Math.max(maxYaw, Math.abs(wrap45(d!.yaw - c.yaw)));
        n++;
      }
    }
    expect(n).toBe(200);
    expect(maxPos).toBeLessThan(2);         // mm
    expect(maxYaw).toBeLessThan(3);         // deg
  });

  it('rejects what is not a cube in the zone: the IR beam, a cube off the zone, two cubes touching', () => {
    const beam = { x: CELL.cube.x, y: 0, yaw: 0, size: [2, 90, 2] as [number, number, number], z0: 11, top: SHADE.RED.top, side: SHADE.RED.side };
    const off = cubeBox({ colour: 'BLUE', x: 330, y: 0, yaw: 0 });
    const pair = [cubeBox({ colour: 'RED', x: 150, y: 100, yaw: 0 }), cubeBox({ colour: 'RED', x: 150, y: 125, yaw: 0 })];
    const all = detect(synthFrame(CAMERA, [beam, off, ...pair]), H);
    expect(all.map(d => d.reject).sort()).toEqual(['large', 'shape', 'zone']);
  });
});

describe('layouts', () => {
  it('scattered cubes are reachable, spaced, clear of the cradle, and replayable from a seed', () => {
    const colours: CubeColour[] = ['RED', 'RED', 'BLUE', 'BLUE', 'RED', 'BLUE'];
    const a = scatterLayout(colours, rng32(7));
    expect(a).toEqual(scatterLayout(colours, rng32(7)));
    expect(a.map(c => c.colour)).toEqual(colours);
    for (const c of a) {
      const roll = graspRoll(c.x, c.y, c.yaw);
      expect(roll).not.toBeNull();
      expect(Math.abs(wrap45(roll! - c.yaw))).toBeLessThan(1e-9);   // fingers square to the faces
      expect(ik({ x: c.x, y: c.y, z: CELL.cube.z, roll: roll! }).status).toBe('ok');
      expect(Math.abs(c.x - CELL.cube.x) >= 35 || Math.abs(c.y) >= 75).toBe(true);
      for (const o of a) if (o !== c) expect(Math.hypot(o.x - c.x, o.y - c.y)).toBeGreaterThanOrEqual(60);
    }
  });

  it('graspRoll turns the wrist the short way and refuses what the arm cannot reach', () => {
    expect(graspRoll(200, 0, 30)).toBe(30);
    expect(graspRoll(200, 0, -40)).toBe(-40);
    const j1 = (Math.atan2(150, 150) * 180) / Math.PI;                      // 45°
    expect(Math.abs(graspRoll(150, 150, -40)! - j1)).toBeLessThanOrEqual(45);
    expect(graspRoll(400, 0, 0)).toBeNull();
  });
});
