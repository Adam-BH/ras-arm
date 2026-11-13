// Find red and blue cubes in a camera frame: HSV threshold → 4-connected blobs → each blob's pixels
// mapped to the table (homography) → centre, yaw, area, elongation → keep the cube-shaped ones in the zone.
import { BLOB, HSV, ZONE } from './config';
import { applyH, type Mat3 } from './geometry';

export type Frame = { width: number; height: number; data: Uint8ClampedArray };   // RGBA, row 0 at the top (ImageData layout)
export type CubeColour = 'RED' | 'BLUE';
export type Reject = 'small' | 'large' | 'shape' | 'zone';
export type Detection = {
  colour: CubeColour;
  x: number; y: number;          // cube centre on the table, mm
  yaw: number;                   // deg in [−45, 45): a square looks the same every 90°
  areaMm2: number; elong: number;
  px: { u: number; v: number; n: number };   // blob centroid in the image, pixel count
  reject?: Reject;
};

export const wrap45 = (deg: number) => ((deg % 90) + 135) % 90 - 45;

/** r, g, b 0..255 → hue deg 0..360, saturation 0..1, value 0..1 */
export function rgbToHsv(r: number, g: number, b: number): [number, number, number] {
  const max = Math.max(r, g, b), min = Math.min(r, g, b), d = max - min;
  let h = 0;
  if (d > 0) {
    if (max === r) h = ((g - b) / d + 6) % 6;
    else if (max === g) h = (b - r) / d + 2;
    else h = (r - g) / d + 4;
  }
  return [h * 60, max ? d / max : 0, max / 255];
}

const inHue = (h: number, [lo, hi]: readonly [number, number]) => (lo <= hi ? h >= lo && h <= hi : h >= lo || h <= hi);

/** 0 = background, 1 = red, 2 = blue */
export function pixelClass(r: number, g: number, b: number): 0 | 1 | 2 {
  const [h, s, v] = rgbToHsv(r, g, b);
  if (s < HSV.minS || v < HSV.minV) return 0;
  if (inHue(h, HSV.red)) return 1;
  if (inHue(h, HSV.blue)) return 2;
  return 0;
}

export function classMap(f: Frame): Uint8Array {
  const out = new Uint8Array(f.width * f.height);
  for (let i = 0, k = 0; i < out.length; i++, k += 4) out[i] = pixelClass(f.data[k], f.data[k + 1], f.data[k + 2]);
  return out;
}

export type Blob = { cls: 1 | 2; pixels: number[] };   // pixel indices into the frame

/** 4-connected components of each class (iterative flood fill) */
export function findBlobs(cls: Uint8Array, w: number): Blob[] {
  const seen = new Uint8Array(cls.length), blobs: Blob[] = [], stack: number[] = [];
  for (let start = 0; start < cls.length; start++) {
    const c = cls[start];
    if (!c || seen[start]) continue;
    const pixels: number[] = [];
    seen[start] = 1; stack.push(start);
    while (stack.length) {
      const i = stack.pop()!, x = i % w;
      pixels.push(i);
      for (const j of [x > 0 ? i - 1 : -1, x < w - 1 ? i + 1 : -1, i - w, i + w]) {
        if (j >= 0 && j < cls.length && !seen[j] && cls[j] === c) { seen[j] = 1; stack.push(j); }
      }
    }
    blobs.push({ cls: c as 1 | 2, pixels });
  }
  return blobs;
}

/** one blob → table centre, yaw, area, elongation. Every pixel is mapped to the table first, so the
 *  numbers come out in mm whatever the camera's position, tilt or orientation. */
export function measure(b: Blob, w: number, H: Mat3): Detection {
  const n = b.pixels.length, X = new Float64Array(n), Y = new Float64Array(n);
  let mx = 0, my = 0, su = 0, sv = 0;
  b.pixels.forEach((i, k) => {
    const u = (i % w) + 0.5, v = Math.floor(i / w) + 0.5;
    [X[k], Y[k]] = applyH(H, [u, v]);
    mx += X[k]; my += Y[k]; su += u; sv += v;
  });
  mx /= n; my /= n;
  // covariance → elongation; 4th harmonic Σ (dx + i·dy)⁴ → yaw: a square's corners repeat every 90°,
  // so the sum points at 180° + 4·yaw whatever the pixel noise (no corner hunting)
  let cxx = 0, cyy = 0, cxy = 0, C = 0, S = 0;
  for (let k = 0; k < n; k++) {
    const dx = X[k] - mx, dy = Y[k] - my, x2 = dx * dx, y2 = dy * dy;
    cxx += x2; cyy += y2; cxy += dx * dy;
    C += x2 * x2 - 6 * x2 * y2 + y2 * y2; S += 4 * dx * dy * (x2 - y2);
  }
  const half = (cxx + cyy) / 2, disc = Math.sqrt(Math.max(0, half * half - (cxx * cyy - cxy * cxy)));
  const elong = Math.sqrt((half + disc) / Math.max(half - disc, 1e-9));
  // area = pixel count × one pixel's footprint on the table at the centroid
  const u = su / n, v = sv / n, p0 = applyH(H, [u, v]), p1 = applyH(H, [u + 1, v]), p2 = applyH(H, [u, v + 1]);
  const pxMm2 = Math.abs((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]));
  const yaw = wrap45(((Math.atan2(S, C) * 180) / Math.PI - 180) / 4);
  return { colour: b.cls === 1 ? 'RED' : 'BLUE', x: mx, y: my, yaw, areaMm2: n * pxMm2, elong, px: { u, v, n } };
}

/** every blob worth a look, with the reason when it is not a cube in the zone */
export function detect(f: Frame, pixelToTable: Mat3, zone = ZONE): Detection[] {
  return findBlobs(classMap(f), f.width)
    .filter(b => b.pixels.length >= BLOB.minPx)
    .map(b => {
      const d = measure(b, f.width, pixelToTable);
      d.reject = d.elong > BLOB.maxElong ? 'shape' : d.areaMm2 < BLOB.minMm2 ? 'small' : d.areaMm2 > BLOB.maxMm2 ? 'large'
        : d.x < zone.xMin || d.x > zone.xMax || d.y < zone.yMin || d.y > zone.yMax ? 'zone' : undefined;
      return d;
    });
}

export const detectCubes = (f: Frame, pixelToTable: Mat3, zone = ZONE) => detect(f, pixelToTable, zone).filter(d => !d.reject);
