# Firmware architecture

Six layers. **Calls only go down.** A command goes one layer at a time; reading a status value
(an angle, a switch, a position) from any lower layer is allowed. Layers 2–6 are plain C99 with no
Arduino include, so the whole robot runs on a PC against a fake HAL (`pio test -e native`).

```
6 App      fsm.c (11 states) · console.c (serial protocol, telemetry)
5 Tasks    tasks.c (homing, colour, pick, place, return, manual moves, teach) · decisions.c ★
4 Motion   kinematics.c (IK/FK) · motion.c (move_to, line, jog, poses)
3 Joints   joints.c (7 joints in degrees: deg ↔ steps / µs, limits, speed limits, servo ramps)
2 Devices  actuators.c (DDA stepper ISR, trapezoid, PCA9685, LED) · sensors.c · serial_link.c · storage.c
1 HAL      hal.h · hal_avr.cpp (UNO, the only C++/Arduino file) · test/hal_fake.c (PC)
```

| Layer | Knows about | Never knows about |
|---|---|---|
| 1 HAL | registers, Timer1, Wire, EEPROM, Serial | what a joint is |
| 2 Devices | steps, µs, debouncing, I²C protocols, CRC | degrees |
| 3 Joints | degrees, limits, which joint is a stepper | Cartesian space |
| 4 Motion | mm, IK, straight lines, saved poses | cubes, colours |
| 5 Tasks | sequences (pick, place, homing) and the rules | serial text |
| 6 App | states, commands, protocol | registers |

## Example: `MOVE 200 0 100`
`console.c` parses it → `fsm_move()` checks the state → `task_move()` → `motion_to()` runs IK →
`joints_move()` clamps, applies speed limits, converts to steps / µs → `steppers_move_to()` loads the
DDA ISR, servo ramps go to the PCA9685 → `hal_step_pulse()` toggles PORTD.

## Documented exceptions (data services, not motion)
- `storage.c` (layer 2, CRC'd EEPROM records) is used by `motion.c` (poses) and `tasks.c` (colour calibration).
- `serial_link.c` and the status LED (layer 2) are driven by the app layer directly.
- `fsm.c` and `console.c` *read* sensors (IR, E-stop, switches) and joint angles directly: status reads.

## Rules that keep it safe
- **Nothing blocks.** Every task is `start()` + `update()` returning RUNNING/DONE/FAILED. The only
  waits are inside the HAL (2 µs step pulse; EEPROM byte writes during `SAVE`/`CAL`, which kick the watchdog).
- **Watchdog 250 ms**, kicked once per `loop()`.
- **E-stop is checked first**, every loop, from any state.
- **All numbers in `config.h`**, including decision defaults. The sim mirrors them in `sim/src/core/config.ts`.
- **RAM**: strings live in flash (`flash.h`, avr-libc `PROGMEM`, plain C); no `malloc`, no `String`,
  no `printf("%f")` (the console formats numbers itself). Build report: RAM 67.3 %, flash 86 %.

## Stepper motion
One Timer1 interrupt at 20 kHz drives all three steppers with a Bresenham DDA: the axis with the most
steps sets the pace, the others step proportionally, so all joints start and finish together.
The main loop only changes the rate (trapezoid: 25 % accelerate, 50 % cruise, 25 % decelerate).
`joints_move()` stretches a move so that no joint exceeds its `JOINT_MAX_DPS`. Servos follow the same
duration with a smooth ramp, refreshed at 50 Hz over I²C.
