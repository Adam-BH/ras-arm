# ras-arm — Robotic Arm firmware (RAS Build Challenge 2026)

Sorts red and blue cubes from a fixed cradle into two bins. Arduino UNO + CNC Shield V3,
3 steppers (TMC2209) + 4 servos (PCA9685), TCS34725 colour sensor, E18 IR, 3 limit switches.

## Architecture: 6 layers, calls only go down

| Layer | Folder | Job |
|---|---|---|
| 6 App | `src/6_app/` | `fsm` (11 states) · `console` (serial protocol) |
| 5 Tasks | `src/5_tasks/` | `tasks` (homing, colour, pick, place, teach) · `decisions` (**all the tunable logic**) |
| 4 Motion | `src/4_motion/` | `kinematics` (IK/FK) · `motion` (moves, straight lines, saved poses) |
| 3 Joints | `src/3_joints/` | `joints`: 7 joints in degrees (stepper or servo, hidden) |
| 2 Devices | `src/2_devices/` | `actuators` (steppers, servos) · `sensors` (switches, E-stop, IR, colour) · `serial_link` |
| 1 HAL | `src/1_hal/` | `hal.h` contract · `hal_avr.cpp` (UNO) · `test/hal_fake.c` (PC) |

All numbers (pins, lengths, limits, speeds) live in `src/config.h`.

## Build & run
```
pio run -e uno -t upload      # flash the arm
pio device monitor            # type HOME, RUN, TEACH, JOG 0 0 -5, SAVE CUBE, ...
pio test -e native            # all tests on your PC (fake robot)
```

## Serial commands
`HOME` `RUN` `STOP` `RESET` `TEACH` `DONE` `JOG dx dy dz [droll]` `SAVE|GO CUBE|BIN_A|BIN_B|HOME` `CAL RED|BLUE|EMPTY` `STATUS`
Telemetry (20 Hz): `T,ms,state,j1..j6(×10),grip(×10),x,y,z,ir,colour`

## Teach mode UI
`sim/`: React page that connects to the arm over USB (Web Serial), jogs it and saves poses, with a 3D twin.
