// Where the gripper goes for a loose cube: a roll that lines the fingers up with it (any multiple of
// 90° works for a square), picked to keep J6 in range and checked with the same IK as the firmware.
// Also the random, reachable, well-spaced layouts the virtual world scatters on the table.
import { CELL, SAFE_Z } from '../core/config';
import { ik } from '../core/kinematics';
import type { LooseCube } from '../core/virtualFirmware';
import { SCATTER, ZONE } from './config';
import type { CubeColour } from './detect';

/** roll (deg, world yaw of the fingers) to grasp a cube at x, y turned by yaw, or null when out of reach */
export function graspRoll(x: number, y: number, yaw: number, gripZ: number = CELL.cube.z): number | null {
  const j1 = (Math.atan2(y, x) * 180) / Math.PI;
  const rolls = [-2, -1, 0, 1, 2].map(k => yaw + 90 * k).sort((a, b) => Math.abs(a - j1) - Math.abs(b - j1));
  for (const roll of rolls)   // J6 = roll − J1: smallest wrist turn first
    if (ik({ x, y, z: gripZ, roll }).status === 'ok' && ik({ x, y, z: SAFE_Z, roll }).status === 'ok') return roll;
  return null;
}

/** seeded PRNG (mulberry32): tests and demos can replay a layout */
export function rng32(seed: number) {
  return () => {
    seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// the cradle, its IR beam (drawn red when clear) and the sensor next to it: keep loose cubes away
const cradleArea = (x: number, y: number) => Math.abs(x - CELL.cube.x) < 35 && Math.abs(y - CELL.cube.y) < 75;

/** random layout: inside the zone, reachable, clear of the cradle and of each other */
export function scatterLayout(colours: CubeColour[], rand: () => number = Math.random): LooseCube[] {
  const out: LooseCube[] = [], e = SCATTER.edgeMm;
  for (let tries = 0; out.length < colours.length && tries < 5000; tries++) {
    const x = Math.round(ZONE.xMin + e + rand() * (ZONE.xMax - ZONE.xMin - 2 * e));
    const y = Math.round(ZONE.yMin + e + rand() * (ZONE.yMax - ZONE.yMin - 2 * e));
    const yaw = Math.round(rand() * 90 - 45);
    if (cradleArea(x, y) || out.some(c => Math.hypot(c.x - x, c.y - y) < SCATTER.minGapMm)) continue;
    if (graspRoll(x, y, yaw) === null) continue;
    out.push({ colour: colours[out.length], x, y, yaw });
  }
  return out;
}
