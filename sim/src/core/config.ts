// Mirror of firmware/src/config.h (geometry, limits, workcell, speeds). Keep both in sync.
export const L1 = 165, L2 = 165, L3 = 110, H_SHOULDER = 120;   // v2 design
export const BASE_MAX_ACC_DPS2 = 143;                          // J1 accel cap (2.5 rad/s²)
export const IK_MAX_COS3 = 0.995, IK_MIN_COS3 = -1;

//                       J1    J2   J3   J4    J5   J6  grip
export const JOINT_MIN = [-100, -20, 20, -90, -100, -90, 0];
export const JOINT_MAX = [100, 110, 150, 90, 100, 90, 50];
export const JOINT_MAX_DPS = [90, 60, 90, 180, 180, 180, 100];
export const JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'Grip'];

export const HOME_SWITCH_DEG = [-100, -20, 150];
export const HOME_DIR = [-1, -1, 1];
export const HOMING = { fast: 30, slow: 4, backoff: 5, leaveMax: 15, margin: 10, slowMax: 8 };

export const CELL = {
  home: { x: 160, y: 0, z: 170, roll: 0 },
  cube: { x: 220, y: 0, z: 24, roll: 0 },
  binA: { x: 0, y: 220, z: 78, roll: 0 },   // red
  binB: { x: 0, y: -220, z: 78, roll: 0 },  // blue
  cubeSize: 25,
};
export const SAFE_Z = 110, PLACE_ABOVE_DZ = 20;
export const GRIP_OPEN_MM = 45, GRIP_CLOSED_MM = 26;
export const RAMP = 0.25;
export const MOVE = { home: 1500, transit: 1600, above: 1200, jog: 300, grip: 300, wiggle: 150 };
export const LINE = { stepMm: 10, maxWp: 40, defaultMmS: 50, pickDown: 40, pickUp: 60, placeDown: 60, placeUp: 80 };
export const TIMING = { boot: 500, irStable: 200, colourSample: 26, colourSamples: 5, telemetry: 50 };
export const PICK_RETRIES = 2, COLOUR_TRIES = 3, REHOME_EVERY = 10;
export const FW_VERSION = '1.0.0';
