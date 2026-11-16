# Teach mode

Touch up the workcell stations (`HOME`, `CUBE`, `BIN_A`, `BIN_B`, and any you added, e.g. `REJECT`) on the
real arm instead of trusting the CAD numbers or the sim layout. Stations live in a table stored in EEPROM
with a CRC; a blank, corrupt or old-format EEPROM falls back to the CAD layout. Every station is validated
by the firmware (`ERR SAVE LIMIT`, `ERR ST TOO_CLOSE`…, see [serial_protocol.md](serial_protocol.md)).

## With the React app (recommended)
1. `cd sim && npm i && npm run dev`, open the page in **Chrome or Edge** (Web Serial).
2. Top bar: **USB** → **Connect** → pick the Arduino. The UNO reboots on connect (≈ 2 s); the app pings
   until it answers and shows `fw 1.0.0`. The arm homes by itself after boot.
3. Teach panel → **Enter TEACH**.
4. Jog with X±/Y±/Z±/Roll± (step 10 / 1 / 0.2 mm), or per joint with J1…J6/Grip −/+.
   Each click sends one `JOG`/`JOGJ`; a click while the arm still moves answers `ERR JOG busy`. Just click again.
5. Put a cube in the cradle, jog the open fingers around it at grip height → **Save** on the `CUBE` row.
6. Same for `BIN_A` (red bin), `BIN_B` (blue bin), `HOME`. **Go** checks a pose, **Preview** comes down onto
   it slowly from its approach height. `SAVE` commits at once; **Add station here** + **Commit** creates new ones.
   The colour → bin dropdowns change the map on the arm (`ST MAP`).
7. Colour calibration, cube on the sensor: **CAL RED**, **CAL BLUE**, empty cradle **CAL EMPTY**.
8. **Leave TEACH** → Run panel → **RUN**.

## With a serial monitor
`pio device monitor` (115200), then for example:
```
TEACH
JOG 0 0 -10
JOG 2 0 0
SAVE CUBE
GO HOME
ST ADD REJECT BIN
ST MAP UNKNOWN REJECT
ST COMMIT
ST LIST
CAL RED
DONE
```

## Manual test notes: SerialLink against the real board
Run these once on the bench after flashing (`pio run -e uno -t upload`). Arm powered, E-stop released.

| # | Do | Expect |
|---|---|---|
| 1 | Connect (USB) | status "USB port open", then `fw 1.0.0` badge; state BOOT → HOMING → IDLE; `EV HOMED` in the console |
| 2 | Type `STATUS` | `OK STATUS IDLE NONE homed=1 running=0` |
| 3 | Move panel: x 200 y 0 z 100 → Move | `OK MOVE`, arm moves, `DONE MOVE`; the 3D twin follows |
| 4 | x 600 | Move button disabled, "unreachable" shown (TS IK); typed `MOVE 600 0 0` → `ERR MOVE unreachable` |
| 5 | Enter TEACH, jog Z− 10 mm ×3 | three `OK JOG` + `DONE JOG`; tip z drops by 30 in telemetry |
| 6 | Click Z− very fast | some `ERR JOG busy`, never a lost reply or a timeout |
| 7 | Save CUBE, power-cycle the UNO, reconnect, `POSES` | CUBE keeps the taught value |
| 8 | Unplug the USB cable | status "USB cable unplugged: press Reconnect", pending commands fail |
| 9 | Plug back, **Reconnect** | reopens without the port picker, board reboots and homes |
| 10 | Press the E-stop during a move | state ESTOP, LED solid; `RESET` → `ERR RESET estop` until released; release → `RESET` → homing |
| 11 | Disconnect one limit switch wire, `HOME` | `EV HOME_FAIL broken_wire <n>`, ERROR, LED 1 blink, the joint only moved *away* from its switch |
| 12 | Cube in the cradle, `RUN` | READ_COLOUR → PICK → VERIFY → PLACE → RETURN → IDLE, `EV CYCLE 1` |

Not yet run on hardware: the arm is not built yet. The same flows are covered automatically on the fake
robot (`pio test -e native`) and on the virtual twin (`npm test`).
