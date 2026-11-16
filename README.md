# ras-arm: cube-sorting robotic arm (IEEE RAS Build Challenge 2026, Project 3)

A 6-axis arm that picks red and blue cubes from a fixed cradle and sorts them into two bins.
Arduino UNO + CNC Shield V3 drive 3 steppers (TMC2209) and 4 servos (PCA9685); a TCS34725 reads the
colour, an E18 IR beam sees the cube arrive, NC limit switches home the arm, a latching E-stop cuts motor power.

This repo holds the **firmware** (C, PlatformIO, 6 strict layers, 24 PC tests) and **pysim**, a physics
simulation of the whole cell (PyBullet): the design book's arm, a Python twin of the firmware, and the
board's serial port, so any serial tool can drive the simulated arm as if it were the UNO.

> 📷 *Photo of the arm: placeholder* · 🗺️ *Workcell diagram: placeholder*

```
           Bin A (red)  (0, +220)
                 ▲ +y
                 │
      base ●─────┼──────▶ cradle (220, 0)   +x
                 │        IR beam + colour sensor
                 ▼
           Bin B (blue) (0, −220)
```

## Architecture

| Layer | Folder | Job |
|---|---|---|
| 6 App | `firmware/src/6_app/` | `fsm` (11 states) · `console` (serial protocol, telemetry) |
| 5 Tasks | `firmware/src/5_tasks/` | `tasks` (homing, colour, pick, place, teach) · `decisions` ★ all tunable rules |
| 4 Motion | `firmware/src/4_motion/` | `kinematics` (IK/FK) · `motion` (moves, straight lines, saved poses) |
| 3 Joints | `firmware/src/3_joints/` | 7 joints in degrees (stepper or servo hidden), limits, speed limits |
| 2 Devices | `firmware/src/2_devices/` | steppers (DDA ISR, trapezoid) + servos + LED · sensors · serial link · EEPROM storage |
| 1 HAL | `firmware/src/1_hal/` | `hal.h` contract · `hal_avr.cpp` (UNO) · `test/test_arm/hal_fake.c` (PC) |

Calls only go down, one layer at a time; all numbers live in `config.h`. Details: [docs/architecture.md](docs/architecture.md).

**State machine**: `BOOT → HOMING → IDLE → READ_COLOUR → PICK → VERIFY → PLACE → RETURN → IDLE`, plus
`TEACH`, `ERROR`, `ESTOP`. Failed pick → jiggle retry (×2). Re-home every 10 cycles. → [docs/state_machine.md](docs/state_machine.md)

## Wiring & pinout (summary)

| Signal | Pin | | Signal | Pin |
|---|---|---|---|---|
| J1 STEP/DIR | D2/D5 | | Limit J1/J2/J3 (NC) | D9/D10/D11 (D12 option) |
| J2 STEP/DIR | D3/D6 | | E-stop sense | A0 |
| J3 STEP/DIR | D4/D7 | | E18 IR | A3 |
| Drivers EN | D8 (LOW = on) | | I²C (PCA9685 0x40, TCS34725 0x29) | A4/A5 |
| Status LED | D13 | | Servos J4/J5/J6/grip | PCA9685 ch 3/0/1/2 (rev F) |

Full table, power and LED codes: [docs/wiring_pinout.md](docs/wiring_pinout.md).

## Serial protocol (summary)
115200 baud, one ASCII command per line, exactly one `OK <CMD>` / `ERR <CMD> <reason>` reply each,
`DONE <CMD>` when a move ends, `EV …` events, 20 Hz telemetry:
```
T,<ms>,<state>,<j1..j6 ×10>,<grip ×10>,<x>,<y>,<z>,<ir>,<colour>,<err>,<lim1><lim2><lim3>,<estop>
```
Commands: `PING STATUS HOME RUN STOP RESET TEACH DONE MOVE LINE MOVEJ JOG JOGJ GRIP SAVE GO POSES CAL SET`.
Full spec: [docs/serial_protocol.md](docs/serial_protocol.md).

## Firmware: build, flash, test
Needs [PlatformIO](https://platformio.org/install/cli) (`pipx install platformio`).
```bash
cd firmware
pio run -e uno                 # build (prints RAM / flash: 67.5 % / 85.6 %)
pio run -e uno -t upload       # flash the UNO
pio device monitor             # talk to it: PING, STATUS, TEACH, JOG 0 0 -5, SAVE CUBE, RUN …
pio test -e native             # 24 tests on your PC against a fake robot
```
Shield clone with Z− on D12? Add `-DLIMIT_Z_ON_D12` to `build_flags` in `platformio.ini`.

## Teach mode in 30 seconds
`TEACH` → `JOG` the open gripper around a cube in the cradle → `SAVE CUBE` → same for `BIN_A`,
`BIN_B`, `HOME` → `CAL RED` / `CAL BLUE` / `CAL EMPTY` with the cube on the sensor → `DONE` → `RUN`.
Walkthrough and bench test checklist: [docs/teach_mode.md](docs/teach_mode.md).

## Folder map
```
firmware/            PlatformIO project (env uno, env native)
  src/config.h       every number: pins, geometry, limits, speeds, timings, decision defaults
  src/1_hal … 6_app  the six layers
  test/test_arm/     fake robot (hal_fake.c) + Unity tests + generated IK vectors
pysim/               PyBullet physics sim: the design book's arm from config.h, Python firmware twin, serial
                     port like the UNO, vision sort, keyframes, GUI + console, pytest (CI job pysim)
docs/                architecture, state machine, kinematics, wiring, protocol, teach mode, decisions
tools/               ik_reference.py (golden vectors), link_length_sweep.py (L1/L2 torque sweep)
.github/workflows/   CI: golden IK vectors, build uno, native tests, pysim tests + headless demo
```

## Kinematics, decisions
- IK/FK with the gripper kept vertical, conventions and stations: [docs/kinematics.md](docs/kinematics.md)
- Every decision rule (colour, 2-colour trick, jiggle, wiggle, pick-OK, re-home) and its parameter: [docs/decisions.md](docs/decisions.md)

## Physics simulation (pysim/)
The cell in PyBullet: the design book's arm generated from `config.h` (tower, belts, servos, gripper, Table 3
masses), a 35 mm cube on the V-cradle, steppers that can lose steps, servos, NC switches, the IR beam, a
TCS34725 model, an E-stop that makes the arm go limp, and a Python twin of the firmware (same states,
decisions and protocol) driving it. Grasps, slips and drops are physics, not script. The twin is reachable on
a serial port with the UNO's exact framing and auto-reset (`--serial tcp:5005`, or a COM port with com0com).
On top, over the normal protocol: the RUN cycle with a cradle feeder, a vision sort (overhead camera → OpenCV
HSV → depth unprojection → pick anywhere on the table), and an ICRA-style keyframe player.
```bash
cd pysim && pip install -r requirements.txt
python run_sim.py                      # window with buttons + protocol console (!help)
python run_sim.py demo --headless      # RUN cycle, vision sort, keyframes; exit 1 if a cube ends up wrong
python run_sim.py --serial tcp:5005    # + the board's serial port: pio device monitor --port socket://localhost:5005
pytest tests                           # 31 tests: model vs firmware and design book, faults, serial port, vision
```
Details, the sensor models and what the physics found: [pysim/README.md](pysim/README.md).

<img src="pysim/img/workcell.png" width="620" alt="PyBullet workcell: the design book's arm lifting a blue cube off the V-cradle">

MIT licence.
