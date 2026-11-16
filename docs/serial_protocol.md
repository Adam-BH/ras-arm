# Serial protocol (board ↔ PC / React app)

The contract between `firmware/src/6_app/console.c`, the virtual twin `sim/src/core/virtualFirmware.ts`
and the app parser `sim/src/link/protocol.ts`. Tests on both sides check it (`test_every_command_one_reply`,
`test_telemetry_format`, `protocol.test.ts`, `virtualLink.test.ts`).

- USB serial, **115200 baud**, 8N1, ASCII lines ending in `\n` (`\r` is ignored).
- Commands are case-insensitive (`move 200 0 100` works). Max line length: 71 characters.
- **Every command gets exactly one reply**: `OK <CMD> [data]` or `ERR <CMD> <reason>`.
- Moves (and `HOME`, `CAL`) also send `DONE <CMD>` when they finish (also after `STOP` interrupts them).
  No `DONE` comes if an E-stop or an error interrupts the move.
- One move at a time: a move command while another runs gets `ERR <CMD> busy`.
- Lines starting with `EV ` are events, `T,` telemetry, `POSE ` / `ST,` / `MAP,` data. An empty line is ignored (no reply).

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
| `SAVE <name>` / `GO <name>` | TEACH | any station name. Save the current tip pose to that station (validated, **committed** to EEPROM at once) / move straight there |
| `POSES` | any | one `POSE <name> x y z roll` line per station (0.1 mm), then `OK POSES` |
| `ST …` | see below | the workcell layout (station table + colour → bin map) |
| `CAL RED\|BLUE\|EMPTY` | TEACH | average 5 colour readings, store as the reference (EEPROM), `EV CAL <colour>`, `DONE CAL` |
| `SET <key> <value>` | IDLE, TEACH | runtime tuning of a decision parameter (not saved). Keys in [decisions.md](decisions.md) |

### Workcell layout: `ST` commands

Edits change RAM only, are accepted **only in TEACH** (except `ST LIST`), and are validated by the firmware.
`ST COMMIT` writes EEPROM. Leaving TEACH with uncommitted edits sends `EV WARN uncommitted_layout`
(the edits stay active until reset). Names: 1–7 chars `A-Z 0-9 _`, case-insensitive on input.

| Command | Effect |
|---|---|
| `ST LIST` | `ST,<i>,<name>,<type>,<x>,<y>,<z>,<roll>,<above_dz>` per station, then `MAP,<colour>,<name\|->` for RED, BLUE, UNKNOWN, then `OK ST` (integers, mm / deg) |
| `ST ADD <name> <PARK\|PICK\|BIN>` | new station at the **current tip pose** (teach on the arm), above_dz 60 |
| `ST SET <name> <x> <y> <z> <roll> [above_dz [type]]` | move a station, or create it (type default `BIN`). Numbers may have decimals (stored to 0.1) |
| `ST DEL <name>` | delete (`HOME` and the pick station are protected) |
| `ST MAP <RED\|BLUE\|UNKNOWN> <name\|->` | colour → bin (`-` = none). UNKNOWN = the reject bin |
| `ST GO <name>` | preview: move above the station, then a slow line down (20 mm/s); `DONE ST` at the end |
| `ST COMMIT` / `ST REVERT` / `ST FACTORY` | save to EEPROM (layout version + 1) / reload EEPROM / CAD defaults (still needs `COMMIT`) |

Replies: `OK ST`, `ERR ST args` (syntax), or `ERR ST <reason>`:

| Reason | Meaning |
|---|---|
| `UNREACHABLE` | out of reach (grip pose, approach, or the vertical line between them) |
| `SINGULAR` | arm (nearly) straight: cos J3 > 0.995 |
| `LIMIT` | a joint limit, or J1 outside the working range −90…+90° (except PARK stations) |
| `KEEPOUT` | tip below `Z_MIN_TABLE` (10 mm), within `R_MIN_BASE` (90 mm) of the base axis, bin drop below 70 mm, or above_dz < 0 |
| `TOO_CLOSE` | pick/bin centre within `MIN_STATION_GAP` (80 mm) of another (PARK stations have no footprint) |
| `FULL` | 8 stations already, or a second PICK station |
| `NAME` | bad / duplicate / unknown name, `MAP` to something that is not a bin |
| `NOT_TEACH` | edit outside TEACH |
| `PROTECTED` | deleting `HOME` or the pick station |

What is checked for a station: the grip pose, then every 10 mm up to the approach pose
z = max(grip + above_dz, `SAFE_Z`) (PARK stations: the pose only). Each pose: IK solvable with the gripper
vertical (elbow up), not singular, joints in limits, keep-out. The v2 design keeps every motor ≤ 70 % of its
torque in the worst pose (arm stretched horizontally), so a pose that passes is torque-safe too.
`SAVE <name>` failing validation answers `ERR SAVE <reason>` with the same reasons.

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
| `pose` | unknown station name (`SAVE` / `GO`) |
| `key` | unknown `SET` key |
| `estop` | motor power is still off |

## Events

`EV <what> [arg] [n]`

| Event | When |
|---|---|
| `EV BOOT` | after reset |
| `EV WARN no_servo_driver` / `EV WARN no_colour_sensor` | PCA9685 / TCS34725 did not answer at boot |
| `EV WARN uncommitted_layout` | left TEACH with station edits not saved (`ST COMMIT`) |
| `EV HOMED` | homing finished |
| `EV HOME_FAIL <why> <joint>` | why ∈ `broken_wire` `not_found` `stuck` `timeout` `home_pose`, joint 1–3 |
| `EV COLOUR RED\|BLUE\|UNKNOWN` | cube colour decided (UNKNOWN = going to the reject bin) |
| `EV PICK_RETRY <n>` | VERIFY failed, retry n with jiggle |
| `EV PICKED <attempt>` | VERIFY passed |
| `EV PLACED RED\|BLUE\|UNKNOWN` | cube released in its bin (from the colour → bin map) |
| `EV CYCLE <n>` | cycle n finished |
| `EV REHOME` | drift-check homing (every 10 cycles) |
| `EV CAL <colour>` | calibration stored |
| `EV ERROR <name>` | entered ERROR |
| `EV ESTOP` | motor power lost |

## Telemetry (20 Hz)

```
T,<ms>,<state>,<j1>,<j2>,<j3>,<j4>,<j5>,<j6>,<grip>,<x>,<y>,<z>,<ir>,<colour>,<err>,<lim1><lim2><lim3>,<estop>,<layout>
T,123456,PICK,0,470,939,0,391,-40,450,220,0,24,1,1,0,010,0,3
```

19 comma-separated fields, **integers only** (no `printf("%f")` on the UNO). The sim also accepts the
older 18-field line (no layout).

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
| layout | station table version, 0–255, +1 per `ST COMMIT` (also via `SAVE`): re-read `ST LIST` when it changes |

## Example session

```
> PING                      < OK PING ras-arm 1.0.0
> TEACH                     < OK TEACH
> JOG 0 0 -10               < OK JOG        … < DONE JOG
> SAVE CUBE                 < OK SAVE
> ST SET REJECT 150 150 80 45   < OK ST
> ST MAP UNKNOWN REJECT     < OK ST
> ST COMMIT                 < OK ST
> MOVE 600 0 0              < ERR MOVE unreachable
> DONE                      < OK DONE
> RUN                       < OK RUN        … < EV COLOUR RED  < EV PICKED 0  < EV PLACED RED  < EV CYCLE 1
```

## Deviations from the original brief
- `STATUS` and `PING` answer as `OK STATUS …` / `OK PING …` so that *every* command really has one `OK`/`ERR` line.
- `JOGJ 7 <mm>` jogs the gripper (not in the brief, handy in teach mode).
