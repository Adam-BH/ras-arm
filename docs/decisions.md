# Decisions: every rule and its parameter

All tunable logic lives in **`firmware/src/5_tasks/decisions.c`** (pure C, unit-tested). Defaults are in
`config.h`; most can be changed at runtime with `SET <key> <value>` in IDLE or TEACH (not saved:
put the final value in `config.h`). The Python twin (`pysim/ras_sim/firmware.py`) implements the same rules.

| # | Rule | Parameter (`SET` key) | Default |
|---|---|---|---|
| R1 | **Empty**: clear channel < `empty_clear × empty_ratio` → nothing on the sensor | `empty_ratio`; `empty_clear` from `CAL EMPTY` | 1.3; 250 |
| R2 | **Colour**: normalise by brightness (r/c, g/c, b/c), nearest calibrated colour, only if `d_best < confident_ratio × d_other` | `confident_ratio`; refs from `CAL RED` / `CAL BLUE` | 0.67 |
| R3 | **2-colour trick**: only red and blue exist. If R2 is unsure, compare with the last *confirmed* cube: within `same_as_last` → same colour, else the other | `use_last_trick` (1/0), `same_as_last` | 1, 0.08 |
| R4 | **Unknown**: re-read, `COLOUR_TRIES` readings in total, then ERROR | `COLOUR_TRIES` (config.h) | 3 |
| R5 | **Pick OK**: IR beam clear **and** colour sensor EMPTY (both must agree) | — | — |
| R6 | **Retries**: extra picks after a failed VERIFY, then ERROR | `pick_retries` | 2 |
| R7 | **Jiggle**: retry n shifts the grip point: 1 → (+mm, 0, +deg), 2 → (−mm, 0, −deg), 3 → (0, +mm), 4 → (0, −mm) | `jiggle_mm`, `jiggle_deg` | 3 mm, 4° |
| R8 | **Wiggle**: on retries, J6 wiggles +`grip_wiggle` and back while the fingers close | `grip_wiggle` (0 = off) | 5° |
| R9 | **Re-home**: home again every N cycles to catch lost steps | `rehome_every` (0 = never) | 10 |

Colour calibration (R1/R2 references) **is** saved to EEPROM with a CRC; a bad CRC falls back to the
defaults in `config.h`.

## Tuning tips
- Calibrate in the real light: `TEACH`, put a red cube in the cradle, `CAL RED`; same for blue; empty cradle `CAL EMPTY`.
- Lots of `EV PICK_RETRY`? Teach `CUBE` more precisely first, then raise `grip_wiggle` or `jiggle_mm`.
- `COLOUR_UNKNOWN` errors with a well-calibrated sensor: lower `confident_ratio` (e.g. 0.8 accepts closer calls).
