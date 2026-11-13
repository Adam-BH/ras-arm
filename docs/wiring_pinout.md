# Wiring & pinout

Arduino UNO (ATmega328P, 16 MHz, 32 KB flash, 2 KB RAM, 1 KB EEPROM) + CNC Shield V3.
Every pin below is defined once in `firmware/src/config.h`.

| Signal | Pin | Note |
|---|---|---|
| J1 STEP / DIR | D2 / D5 | shield X (PD2 / PD5) |
| J2 STEP / DIR | D3 / D6 | shield Y (PD3 / PD6) |
| J3 STEP / DIR | D4 / D7 | shield Z (PD4 / PD7) |
| Drivers EN | D8 | active LOW, all three TMC2209 |
| Limit J1 / J2 / J3 | D9 / D10 / D11 | X− Y− Z−. Some shields route Z− to **D12**: build with `-DLIMIT_Z_ON_D12` |
| E-stop / motor-rail sense | A0 (14) | HIGH = power OK (47 k / 10 k divider from 24 V if no second contact) |
| TCS34725 LED enable | A1 (15) | driven HIGH at boot (optional) |
| Spare / buzzer | A2 (16) | unused |
| E18-D80NK IR | A3 (17) | INPUT_PULLUP, NPN: LOW = object |
| I²C SDA / SCL | A4 / A5 | PCA9685 @ 0x40 + TCS34725 @ 0x29, 400 kHz, 3 ms bus timeout |
| Status LED | D13 | blink codes (see below) |
| Serial | D0 / D1 | USB, 115200 baud |
| Timer1 | — | stepper ISR at 20 kHz (**do not use `Servo.h`**: it takes Timer1) |

## PCA9685 channels (50 Hz)
| Ch | Joint | Servo |
|---|---|---|
| 0 | J4 forearm roll | DS3218MG |
| 1 | J5 wrist pitch | DS3218MG |
| 2 | J6 wrist roll | MG996 |
| 3 | Gripper | MG996 |

⚠ DS3218MG comes in **180° and 270°** versions. Set `DS3218_RANGE_DEG` in `config.h` to match the label.
Trim each servo with `SERVO_CENTER_US` / `SERVO_US_PER_UNIT` (sign = direction).

## Steppers
| Joint | Motor | Drive | Microstep (shield jumpers) | Steps / joint ° |
|---|---|---|---|---|
| J1 base | 17HS4401 | direct | 1/16 | 8.89 |
| J2 shoulder | 17HS4401S + PG518 (5.18:1) | + GT2 belt 16T → 48T (×3) | 1/8 | 69.07 |
| J3 elbow | 17HS4401 (on the J2 axis) | GT2 belt 16T → 80T (×5) | 1/8 | 22.22 |

A motor turning the wrong way: flip `DIR_SIGN_Jn` in `config.h` (or swap one coil pair).

## Limit switches: why NC
KW4 switches are wired **normally closed** between the pin (INPUT_PULLUP) and GND.
Not pressed → closed → LOW. Pressed **or wire broken** → open → HIGH. A broken wire therefore reads
"pressed": homing first tries to *leave* the switch, and if it is still "pressed" after 15° it stops
with `EV HOME_FAIL broken_wire <joint>` instead of driving into the mechanical stop.

## Power
24 V 12.5 A PSU → 20 A fuse → CNC shield (motors) · XL4016 → ≈ 6.5 V servo rail (PCA9685 V+) ·
LM2596 → 5 V logic. The latching E-stop cuts the motor rail in hardware; the UNO only *senses* it on A0
and stops sending steps (state ESTOP, homing required after `RESET`).

## Status LED codes (D13)
| Pattern | Meaning |
|---|---|
| slow heartbeat (1 s) | healthy |
| 1 blink + pause | HOME_FAILED |
| 2 blinks | COLOUR_UNKNOWN |
| 3 blinks | PICK_FAILED |
| 4 blinks | UNREACHABLE |
| solid | E-stop (motor power off) |
