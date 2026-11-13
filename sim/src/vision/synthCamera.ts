// A software overhead camera (no WebGL) for the tests: boxes projected through the same pinhole model,
// silhouette filled with the side shade, then the top face with the lit shade, on a dark table.
// The shades are what the WebGL capture of the 3D scene gives for the same materials.
import { CELL } from '../core/config';
import { project, type Camera, type Vec2 } from './geometry';
import type { CubeColour, Frame } from './detect';

type RGB = [number, number, number];
export type SynthBox = { x: number; y: number; yaw: number; size: [number, number, number]; z0?: number; top: RGB; side: RGB };

export const SHADE: Record<CubeColour, { top: RGB; side: RGB }> = {
  RED: { top: [158, 33, 33], side: [112, 24, 24] },
  BLUE: { top: [24, 77, 156], side: [17, 55, 112] },
};
const TABLE: RGB = [42, 47, 56];

export const cubeBox = (c: { colour: CubeColour; x: number; y: number; yaw: number }): SynthBox => {
  const s = CELL.cubeSize;
  return { x: c.x, y: c.y, yaw: c.yaw, size: [s, s, s], ...SHADE[c.colour] };
};

/** convex hull, monotone chain */
function hull(pts: Vec2[]): Vec2[] {
  const p = [...pts].sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o: Vec2, a: Vec2, b: Vec2) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const half = (list: Vec2[]) => {
    const h: Vec2[] = [];
    for (const q of list) {
      while (h.length >= 2 && cross(h[h.length - 2], h[h.length - 1], q) <= 0) h.pop();
      h.push(q);
    }
    return h.slice(0, -1);
  };
  return [...half(p), ...half([...p].reverse())];
}

function fill(img: Uint8ClampedArray, w: number, h: number, poly: Vec2[], rgb: RGB) {
  const us = poly.map(p => p[0]), vs = poly.map(p => p[1]);
  const u0 = Math.max(0, Math.floor(Math.min(...us))), u1 = Math.min(w - 1, Math.ceil(Math.max(...us)));
  const v0 = Math.max(0, Math.floor(Math.min(...vs))), v1 = Math.min(h - 1, Math.ceil(Math.max(...vs)));
  for (let v = v0; v <= v1; v++) {
    for (let u = u0; u <= u1; u++) {
      const pu = u + 0.5, pv = v + 0.5;
      let inside = true;
      for (let k = 0; k < poly.length && inside; k++) {
        const [au, av] = poly[k], [bu, bv] = poly[(k + 1) % poly.length];
        inside = (bu - au) * (pv - av) - (bv - av) * (pu - au) >= 0;   // hull is counter-clockwise
      }
      if (inside) img.set(rgb, (v * w + u) * 4);
    }
  }
}

export function synthFrame(cam: Camera, boxes: SynthBox[], noise = 0, rand: () => number = Math.random): Frame {
  const { width: w, height: h } = cam, data = new Uint8ClampedArray(w * h * 4);
  for (let i = 0; i < w * h; i++) data.set([...TABLE, 255], i * 4);
  for (const b of boxes) {
    const c = Math.cos((b.yaw * Math.PI) / 180), s = Math.sin((b.yaw * Math.PI) / 180), z0 = b.z0 ?? 0;
    const corners = [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([i, j]) => {
      const lx = (i * b.size[0]) / 2, ly = (j * b.size[1]) / 2;
      return { x: b.x + c * lx - s * ly, y: b.y + s * lx + c * ly };
    });
    const top = corners.map(p => project(cam, { ...p, z: z0 + b.size[2] }));
    fill(data, w, h, hull([...corners.map(p => project(cam, { ...p, z: z0 })), ...top]), b.side);
    fill(data, w, h, hull(top), b.top);
  }
  if (noise > 0) {
    for (let i = 0; i < data.length; i++) if ((i & 3) !== 3) data[i] += Math.round((rand() * 2 - 1) * noise);
  }
  return { width: w, height: h, data };
}
