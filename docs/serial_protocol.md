# Serial protocol (board ↔ PC)

The contract between `firmware/src/6_app/console.c`, the Python twin `pysim/ras_sim/firmware.py` and its
parser `pysim/ras_sim/protocol.py`. Tests on both sides check it (`test_every_command_one_reply`,
`test_telemetry_format`; `pysim/tests/test_firmware.py`, `test_serial.py`). `pysim/run_sim.py --serial tcp:5005`
serves the twin on a port with this exact framing.

- USB serial, **115200 baud**, 8N1, ASCII lines ending in `\n` (`\r` is ignored).
- Commands are case-insensitive (`move 200 0 100` works). Max line length: 71 characters.
- **Every command gets exactly one reply**: `OK <CMD> [data]` or `ERR <CMD> <reason>`.
- Moves (and `HOME`, `CAL`) also send `DONE <CMD>` when they finish (also after `STOP` interrupts them).
  No `DONE` comes if an E-stop or an error interrupts the move.
- One move at a time: a move command while another runs gets `ERR <CMD> busy`.
- Lines starting with `EV ` are events, `T,` telemetry, `POSE ` data. An empty line is ignored (no reply).

## Commands

| Command | Allowed in | Reply / effect |
|---|---|---|
| `PING` | any | `OK PING ras-arm 1.0.0` |
| `STATUS` | any | `OK STATUS <state> <error> homed=<0/1> running=<0/1>` |
| `HOME` | IDLE, TEACH, ERROR | homing, then `DONE HOME` |
| `RUN` | IDLE | start the sorting loop |
| `STOP` | any | stop the loop after the current cube; also stops a manual move |
| `RESET` | ERROR, ESTOP (power back) | → HOMING |
| `TEACH` / `DONE` | IDLE / TEACH | enter / leave teach mode (`TEACH` also stops `RUN`) |
| `MOVE x y z [roll] [ms]` | IDLE (not running), TEACH | Cartesian move-to via IK, all joints together. `roll` default 0, `ms` 0 = as fast as the speed limits allow |
| `LINE x y z [mm_s]` | IDLE, TEACH | straight line (IK every 10 mm, whole line checked first). Default 50 mm/s |
| `MOVEJ j1 j2 j3 j4 j5 j6 [ms]` | IDLE, TEACH | joint-space move in degrees, limits enforced, gripper untouched |
| `JOG dx dy dz [droll]` | TEACH | relative Cartesian jog (mm, deg) |
| `JOGJ j delta` | TEACH | jog joint `j` (1–6, **7 = gripper** in mm) by `delta` |
| `GRIP mm` | IDLE, TEACH | gripper opening 0–50 mm (clamped) |
| `SAVE <POSE>` / `GO <POSE>` | TEACH | POSE ∈ `HOME CUBE BIN_A BIN_B`. Save the current tip pose to EEPROM / move there |
| `POSES` | any | one `POSE <name> x y z roll` line per pose (0.1 mm), then `OK POSES` |
| `CAL RED\|BLUE\|EMPTY` | TEACH | average 5 colour readings, store as the reference (EEPROM), `EV CAL <colour>`, `DONE CAL` |
| `SET <key> <value>` | IDLE, TEACH | runtime tuning of a decision parameter (not saved). Keys in [decisions.md](decisions.md) |

### Error reasons

| Reason | Meaning |
|---|---|
| `unknown` | not a command |
| `args` | wrong number of arguments, not a number, bad joint index or colour |
| `state` | not allowed in the current state |
| `busy` | another move / homing / calibration is running |
| `running` | the sorting loop is running (`STOP` first) |
| `unreachable` | IK failed (out of reach, or a joint limit) |
| `limit` | `MOVEJ` / `JOGJ` outside the joint limits |
| `pose` | unknown pose name |
| `key` | unknown `SET` key |
| `estop` | motor power is still off |

## Events

`EV <what> [arg] [n]`

| Event | When |
|---|---|
| `EV BOOT` | after reset |
| `EV WARN no_servo_driver` / `EV WARN no_colour_sensor` | PCA9685 / TCS34725 did not answer at boot |
| `EV HOMED` | homing finished |
| `EV HOME_FAIL <why> <joint>` | why ∈ `broken_wire` `not_found` `stuck` `timeout` `home_pose`, joint 1–3 |
| `EV COLOUR RED\|BLUE` | cube colour decided |
| `EV PICK_RETRY <n>` | VERIFY failed, retry n with jiggle |
| `EV PICKED <attempt>` | VERIFY passed |
| `EV PLACED RED\|BLUE` | cube released in its bin |
| `EV CYCLE <n>` | cycle n finished |
| `EV REHOME` | drift-check homing (every 10 cycles) |
| `EV CAL <colour>` | calibration stored |
| `EV ERROR <name>` | entered ERROR |
| `EV ESTOP` | motor power lost |

## Telemetry (20 Hz)

```
T,<ms>,<state>,<j1>,<j2>,<j3>,<j4>,<j5>,<j6>,<grip>,<x>,<y>,<z>,<ir>,<colour>,<err>,<lim1><lim2><lim3>,<estop>
T,123456,PICK,0,470,939,0,391,-40,450,220,0,24,1,1,0,010,0
```

18 comma-separated fields, **integers only** (no `printf("%f")` on the UNO).

| Field | Unit |
|---|---|
| ms | board uptime |
| state | `BOOT HOMING IDLE READ_COLOUR PICK VERIFY PLACE RETURN TEACH ERROR ESTOP` |
| j1…j6, grip | degrees ×10, grip mm ×10 (rounded) |
| x, y, z | tip position in mm (FK, rounded) |
| ir | 1 = beam blocked for 200 ms (cube in the cradle) |
| colour | last colour reading: 0 EMPTY, 1 RED, 2 BLUE, 3 UNKNOWN |
| err | 0 NONE, 1 HOME_FAILED, 2 COLOUR_UNKNOWN, 3 PICK_FAILED, 4 UNREACHABLE, 5 ESTOP (= LED blink count) |
| lim | three digits J1 J2 J3, 1 = switch pressed **or wire broken** (NC wiring) |
| estop | 1 = motor power off |

## Example session

```
> PING                      < OK PING ras-arm 1.0.0
> TEACH                     < OK TEACH
> JOG 0 0 -10               < OK JOG        … < DONE JOG
> SAVE CUBE                 < OK SAVE
> MOVE 600 0 0              < ERR MOVE unreachable
> DONE                      < OK DONE
> RUN                       < OK RUN        … < EV COLOUR RED  < EV PICKED 0  < EV PLACED RED  < EV CYCLE 1
```

## Deviations from the original brief
- `STATUS` and `PING` answer as `OK STATUS …` / `OK PING …` so that *every* command really has one `OK`/`ERR` line.
- `JOGJ 7 <mm>` jogs the gripper (not in the brief, handy in teach mode).
