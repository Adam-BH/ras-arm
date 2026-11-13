// Vision pick-and-place: the overhead camera, the pick zone, detection thresholds, sort timing.
// Units: mm and degrees, robot frame (docs/kinematics.md). Host side only: nothing here is in config.h.
import { CELL } from '../core/config';

// Pinhole camera looking straight down, image up = robot +x, image right = robot −y.
// 400 × 300 px over ≈ 400 × 300 mm of table → ≈ 1 px/mm, a cube top ≈ 26 px wide.
export const CAMERA = { x: 205, y: 0, z: 500, fovDeg: 33.4, width: 400, height: 300 };

export const ZONE = { xMin: 120, xMax: 290, yMin: -160, yMax: 160 };   // loose cubes are looked for here
export const VIEW_POSE = { x: 40, y: -200, z: 170, roll: 0 };          // arm parked out of the picture for each snapshot

// pixels → table on the plane through the cube centres: the side faces a perspective camera
// sees pull the blob toward the image centre, unprojecting at mid-height cancels most of it
export const CUBE_PLANE_Z = CELL.cubeSize / 2;

export const HSV = {
  red: [345, 15] as const,      // hue ranges (deg, red wraps through 0)
  blue: [195, 250] as const,
  minS: 0.45, minV: 0.2,        // the grey table, the orange arm and the purple sensor pad stay out
};
export const BLOB = {
  minPx: 30,                    // specks
  minMm2: 300, maxMm2: 1500,    // a 25 mm cube seen from above: ≈ 625 mm² top + a bit of side
  maxElong: 2.2,                // √(λmax/λmin) of the blob: a square is 1, the thin IR beam is ≫ 2
};

export const SORT = {
  retries: 2,                   // re-picks after a slip (same budget as the firmware's pick_retries)
  verifyMm: 15,                 // still seen this close to where it was → the pick slipped
  settleMs: 150,                // let the scene catch up with the last telemetry before a snapshot
  maxPicks: 16,                 // hard stop for one Sort run
  descendMs: 900, liftMs: 900, placeMs: 900,
};
// pick retries nudge the grasp a little each time (same idea as decisions.c gripOffset): dx, dy mm, droll deg
export const JIGGLE: [number, number, number][] = [[0, 0, 0], [3, 0, 4], [-3, 0, -4], [0, 3, 0], [0, -3, 0]];

export const SCATTER = { minGapMm: 60, cradleClearMm: 50, edgeMm: 20 };
