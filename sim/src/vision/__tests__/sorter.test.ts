// Vision sort end to end: the virtual firmware + a synthetic overhead camera, driven over the protocol.
import { describe, expect, it } from 'vitest';
import { CELL, GRIP_CLOSED_MM, GRIP_OPEN_MM } from '../../core/config';
import type { LooseCube } from '../../core/virtualFirmware';
import { VirtualLink } from '../../link/VirtualLink';
import { CAMERA, CUBE_PLANE_Z } from '../config';
import type { CubeColour } from '../detect';
import { cameraHomography } from '../geometry';
import { rng32, scatterLayout } from '../plan';
import { VisionSorter, type SortStatus } from '../sorter';
import { cubeBox, synthFrame } from '../synthCamera';

async function setup(cubes: LooseCube[]) {
  const link = new VirtualLink({ autoTick: false }), fw = link.fw, events: string[] = [];
  link.onEvent(e => events.push(e));
  await link.connect();
  const pump = async (pred: () => boolean, maxMs = 60000) => {
    for (let t = 0; t < maxMs && !pred(); t += 20) { link.advance(20); for (let k = 0; k < 12; k++) await Promise.resolve(); }
    return pred();
  };
  await pump(() => events.includes('EV HOMED'));
  fw.scatter(cubes);
  const noise = rng32(99), statuses: SortStatus[] = [];
  const sorter = new VisionSorter({
    send: cmd => link.send(cmd).catch(() => null),
    onEvent: cb => link.onEvent(cb),
    capture: async () => synthFrame(CAMERA, [
      ...fw.world.table.map(cubeBox),
      ...(fw.world.cradle ? [cubeBox({ colour: fw.world.cradle, x: CELL.cube.x, y: CELL.cube.y, yaw: 0 })] : []),
    ], 6, noise),
    pixelToTable: cameraHomography(CAMERA, CUBE_PLANE_Z),
    poses: () => fw.poses,
    wait: async () => {},
  }, s => statuses.push(s));
  const run = async (during?: () => void) => {
    let result: SortStatus | null = null;
    const p = sorter.run().then(s => { result = s; });
    if (during) during();
    await pump(() => result !== null, 600000);
    await p;
    return result! as SortStatus;
  };
  return { link, fw, events, sorter, statuses, pump, run };
}

const count = (cubes: LooseCube[], c: CubeColour) => cubes.filter(k => k.colour === c).length;

describe('VisionSorter', () => {
  it('sorts loose cubes into their bins: 3 red + 2 blue anywhere on the table', async () => {
    const cubes = scatterLayout(['RED', 'BLUE', 'RED', 'RED', 'BLUE'], rng32(3));
    const s = await setup(cubes);
    const r = await s.run();
    expect(r.phase).toBe('done');
    expect(r.text).toBe('table clear');
    expect(s.fw.world.table).toEqual([]);
    expect(s.fw.world.held).toBeNull();
    expect(s.fw.world.bins).toEqual({ A: count(cubes, 'RED'), B: count(cubes, 'BLUE') });
    expect(r.bins).toEqual(s.fw.world.bins);
    expect(r.slips).toBe(0);
    expect(s.statuses.some(st => st.phase === 'verify')).toBe(true);
  });

  it('a slipped grasp is seen by the camera and re-picked', async () => {
    const s = await setup(scatterLayout(['BLUE', 'RED'], rng32(11)));
    s.fw.failNextPick();
    const r = await s.run();
    expect(r.phase).toBe('done');
    expect(r.slips).toBe(1);
    expect(s.fw.world.bins).toEqual({ A: 1, B: 1 });
    expect(s.statuses.some(st => st.phase === 'pick' && st.attempt === 1)).toBe(true);
  });

  it('also takes the cube sitting in the cradle', async () => {
    const s = await setup([]);
    s.fw.placeCube('RED');
    const r = await s.run();
    expect(r.phase).toBe('done');
    expect(s.fw.world.cradle).toBeNull();
    expect(s.fw.world.bins).toEqual({ A: 1, B: 0 });
  });

  it('refuses to start while the arm is running', async () => {
    const s = await setup(scatterLayout(['RED'], rng32(5)));
    await s.link.send('RUN').catch(() => null);
    const r = await s.run();
    expect(r.phase).toBe('error');
    expect(r.text).toContain('running=1');
    expect(s.fw.world.table).toHaveLength(1);
  });

  it('E-stop mid-sort ends the run with an error', async () => {
    const s = await setup(scatterLayout(['RED', 'BLUE'], rng32(8)));
    const r = await s.run(() => {
      const off = s.link.onEvent(e => { if (e === 'DONE GRIP') { off(); s.fw.setPower(false); } });
    });
    expect(r.phase).toBe('error');
    expect(r.text).toBe('estop');
  });

  it('stop() halts the arm and ends the run', async () => {
    const s = await setup(scatterLayout(['RED', 'BLUE'], rng32(9)));
    const r = await s.run(() => {
      const off = s.link.onEvent(e => { if (e === 'DONE MOVE') { off(); s.sorter.stop(); } });
    });
    expect(r.phase).toBe('stopped');
    expect(s.fw.world.table.length + s.fw.world.bins.A + s.fw.world.bins.B).toBe(2);
  });
});

describe('virtual world: loose cubes', () => {
  async function arm(cube: LooseCube) {
    const s = await setup([cube]);
    const go = async (cmd: string) => {
      await s.link.send(cmd);
      const name = cmd.split(' ')[0];
      await s.pump(() => s.events.filter(e => e === `DONE ${name}`).length > 0);
      s.events.length = 0;
    };
    return { ...s, go };
  }

  it('fingers square to the cube hold it; 30° off they do not', async () => {
    const s = await arm({ colour: 'RED', x: 200, y: 80, yaw: 20 });
    await s.go('MOVE 200 80 24 50');
    await s.go(`GRIP ${GRIP_CLOSED_MM}`);
    expect(s.fw.world.held).toBeNull();
    await s.go(`GRIP ${GRIP_OPEN_MM}`);
    await s.go('MOVE 200 80 24 110');                // 20 + 90: same faces, J6 still in range
    await s.go(`GRIP ${GRIP_CLOSED_MM}`);
    expect(s.fw.world.held).toBe('RED');
    expect(s.fw.world.table).toEqual([]);
  });

  it('a cube let go over the table lands under the gripper', async () => {
    const s = await arm({ colour: 'BLUE', x: 150, y: -90, yaw: 0 });
    await s.go('MOVE 150 -90 24 0');
    await s.go(`GRIP ${GRIP_CLOSED_MM}`);
    await s.go('MOVE 250 60 60 10');
    await s.go(`GRIP ${GRIP_OPEN_MM}`);
    expect(s.fw.world.held).toBeNull();
    expect(s.fw.world.table).toHaveLength(1);
    expect(s.fw.world.table[0]).toMatchObject({ colour: 'BLUE', yaw: 10 });
    expect(s.fw.world.table[0].x).toBeCloseTo(250, 0);
    expect(s.fw.world.table[0].y).toBeCloseTo(60, 0);
  });
});
