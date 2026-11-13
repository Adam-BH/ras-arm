// Camera geometry: the pinhole model of the overhead camera and pixel ↔ table homographies.
// Pixels: u right, v down, (0, 0) = top-left corner of the image, pixel centres at +0.5.
// A homography maps image pixels to table mm on one horizontal plane. The virtual camera's comes
// from the pinhole model; a real webcam's will come from 4 (or more) clicked reference points.

export type Vec2 = [number, number];
export type Mat3 = number[];                     // 3 × 3, row-major
export type Camera = { x: number; y: number; z: number; fovDeg: number; width: number; height: number };

/** focal length in pixels (fovDeg is the vertical field of view) */
export const focalPx = (cam: Camera) => cam.height / 2 / Math.tan((cam.fovDeg * Math.PI) / 360);

/** robot point (mm) → pixel, for a camera looking straight down with image up = robot +x */
export function project(cam: Camera, p: { x: number; y: number; z: number }): Vec2 {
  const f = focalPx(cam), depth = cam.z - p.z;
  return [cam.width / 2 - (f * (p.y - cam.y)) / depth, cam.height / 2 - (f * (p.x - cam.x)) / depth];
}

export function applyH(H: Mat3, [x, y]: Vec2): Vec2 {
  const w = H[6] * x + H[7] * y + H[8];
  return [(H[0] * x + H[1] * y + H[2]) / w, (H[3] * x + H[4] * y + H[5]) / w];
}

/** least-squares homography src → dst (exact for 4 points, no 3 of them on a line) */
export function solveHomography(src: Vec2[], dst: Vec2[]): Mat3 {
  if (src.length < 4 || src.length !== dst.length) throw new Error('homography: need ≥ 4 point pairs');
  // normal equations AᵀA h = Aᵀb for h0..h7 (h8 = 1)
  const M = Array.from({ length: 8 }, () => new Array<number>(9).fill(0));
  const add = (row: number[], rhs: number) => {
    for (let i = 0; i < 8; i++) {
      for (let j = 0; j < 8; j++) M[i][j] += row[i] * row[j];
      M[i][8] += row[i] * rhs;
    }
  };
  src.forEach(([x, y], k) => {
    const [X, Y] = dst[k];
    add([x, y, 1, 0, 0, 0, -x * X, -y * X], X);
    add([0, 0, 0, x, y, 1, -x * Y, -y * Y], Y);
  });
  // Gauss-Jordan with partial pivoting
  for (let c = 0; c < 8; c++) {
    let p = c;
    for (let r = c + 1; r < 8; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-12) throw new Error('homography: degenerate points');
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = 0; r < 8; r++) {
      if (r === c) continue;
      const k = M[r][c] / M[c][c];
      for (let j = c; j < 9; j++) M[r][j] -= k * M[c][j];
    }
  }
  return [...M.map((row, i) => row[8] / M[i][i]), 1];
}

export function invertH(H: Mat3): Mat3 {
  const [a, b, c, d, e, f, g, h, i] = H;
  const A = e * i - f * h, B = f * g - d * i, C = d * h - e * g;
  const det = a * A + b * B + c * C;
  if (Math.abs(det) < 1e-12) throw new Error('homography: not invertible');
  return [A, c * h - b * i, b * f - c * e, B, a * i - c * g, c * d - a * f, C, b * g - a * h, a * e - b * d].map(v => v / det);
}

/** pixel → table (mm) on the plane z, for the virtual camera */
export function cameraHomography(cam: Camera, z: number): Mat3 {
  const table: Vec2[] = [[cam.x - 100, cam.y - 100], [cam.x + 100, cam.y - 100], [cam.x + 100, cam.y + 100], [cam.x - 100, cam.y + 100]];
  return solveHomography(table.map(([x, y]) => project(cam, { x, y, z })), table);
}
