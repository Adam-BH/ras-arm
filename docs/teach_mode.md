# Teach mode

Teach the four poses the cycle uses (`HOME`, `CUBE`, `BIN_A`, `BIN_B`) on the real arm instead of
trusting the CAD numbers. Poses are stored in EEPROM with a CRC and survive power cycles; a blank or
corrupt EEPROM falls back to the CAD layout.

## With a serial monitor
`pio device monitor` (115200). The UNO reboots when the monitor opens the port (≈ 2 s) and homes by
itself. Then, for example:
```
TEACH
JOG 0 0 -10
JOG 2 0 0
SAVE CUBE
GO HOME
POSES
CAL RED
DONE
```
1. `TEACH`.
2. Jog in mm with `JOG dx dy dz [droll]`, or per joint with `JOGJ <1..7> <deg or mm>`. A jog while the arm still
   moves answers `ERR JOG busy`: send it again after `DONE JOG`.
3. Put a cube in the cradle, jog the open fingers around it at grip height → `SAVE CUBE`.
4. Same for `BIN_A` (red bin), `BIN_B` (blue bin), `HOME`. `GO <pose>` checks a pose.
5. Colour calibration, cube on the sensor: `CAL RED`, `CAL BLUE`, empty cradle `CAL EMPTY`.
6. `DONE` → `RUN`.

## Rehearse it on pysim (no arm needed)
```bash
cd pysim && python run_sim.py --serial tcp:5005
pio device monitor --port socket://localhost:5005 --baud 115200
```
Same commands, same replies: the simulated board resets when the monitor opens the port, homes, and takes
`TEACH` / `JOG` / `SAVE` / `CAL` like the UNO. `!red` in the sim's terminal (or the window's button) puts a
cube in the cradle for `CAL RED`. Details: [pysim/README.md](../pysim/README.md).

## Manual test notes: the real board over USB
Run these once on the bench after flashing (`pio run -e uno -t upload`). Arm powered, E-stop released.

| # | Do | Expect |
|---|---|---|
| 1 | Open the monitor with the arm resting at HOME | `EV BOOT` after ≈ 2 s, then homing and `EV HOMED`. Watch how far the arm sags while the UNO reboots (EN pulled up, steppers off): pysim predicts ≈ 40 mm |
| 2 | `STATUS` | `OK STATUS IDLE NONE homed=1 running=0` |
| 3 | `MOVE 200 0 100` | `OK MOVE`, arm moves, `DONE MOVE`; telemetry tip 200, 0, 100 |
| 4 | `MOVE 600 0 0` | `ERR MOVE unreachable` |
| 5 | `TEACH`, then `JOG 0 0 -10` ×3 | three `OK JOG` + `DONE JOG`; tip z drops by 30 in telemetry |
| 6 | `JOG 0 0 -10` twice in a row, fast | the second answers `ERR JOG busy`, never a lost reply |
| 7 | `SAVE CUBE`, power-cycle the UNO, reopen the monitor, `POSES` | CUBE keeps the taught value |
| 8 | Unplug the USB cable, plug it back, reopen | the board reboots and homes |
| 9 | Press the E-stop during a move | state ESTOP, LED solid; `RESET` → `ERR RESET estop` until released; release → `RESET` → homing |
| 10 | Disconnect one limit switch wire, `HOME` | `EV HOME_FAIL broken_wire <n>`, ERROR, LED 1 blink, the joint only moved *away* from its switch |
| 11 | Cube in the cradle, `RUN` | READ_COLOUR → PICK → VERIFY → PLACE → RETURN → IDLE, `EV CYCLE 1` |

Not yet run on hardware: the arm is not built yet. The same flows are covered automatically on the fake
robot (`pio test -e native`) and on pysim (`pytest pysim/tests`).
