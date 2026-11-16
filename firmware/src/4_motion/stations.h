// LAYER 4 — MOTION · stations (the workcell as DATA: pick cradle, bins, park poses)
// A table of named tip poses + a colour → bin map, in RAM, saved to EEPROM on commit.
// Every station is validated here (reach, singularity, joint limits, keep-out, spacing):
// the firmware is the final authority, whatever the sim or the teach pendant sends.
// Invariants: index 0 = HOME (park), the first STN_PICK station exists; neither can be deleted.
#pragma once
#include "kinematics.h"

typedef enum { STN_PARK, STN_PICK, STN_BIN } station_type_t;   // (ST_PICK is an FSM state)
#define STATION_TYPES "PARK\0PICK\0BIN"
#define STATION_HOME  0

typedef struct {
  char    name[STATION_NAME_LEN];  // "HOME", "CUBE", "BIN_A"… upper case, no spaces
  uint8_t type;                    // station_type_t
  int16_t x, y, z, roll;           // grip / drop tip pose in 0.1 mm / 0.1° (int16, not float: RAM)
  int16_t above_dz;                // approach offset above the grip pose, mm
} station_t;

// reason codes (0 = OK) → "ERR ST <reason>"
typedef enum { STE_OK, STE_UNREACHABLE, STE_SINGULAR, STE_LIMIT, STE_KEEPOUT, STE_TOO_CLOSE,
               STE_FULL, STE_NAME, STE_NOT_TEACH, STE_PROTECTED } st_err_t;
#define ST_ERR_NAMES "OK\0UNREACHABLE\0SINGULAR\0LIMIT\0KEEPOUT\0TOO_CLOSE\0FULL\0NAME\0NOT_TEACH\0PROTECTED"

void             stations_init(void);                 // EEPROM, or CAD defaults if blank/corrupt/old
uint8_t          stations_count(void);
const station_t *stations_get(uint8_t i);
pose_t           stations_grip(uint8_t i);            // grip pose, float
pose_t           stations_above(uint8_t i);           // approach: z = max(grip + above_dz, SAFE_Z); park: = grip
int8_t           stations_find(const char *name);     // -1 = none
int8_t           stations_find_type(uint8_t type, uint8_t nth);

// edits change RAM only (stations_dirty() = true until commit)
st_err_t stations_add(const char *name, uint8_t type, const pose_t *p, int16_t above_dz);
st_err_t stations_set(uint8_t i, const pose_t *p, int16_t above_dz);
st_err_t stations_delete(uint8_t i);
st_err_t stations_map(colour_t c, int8_t i);          // i = -1: no bin for this colour
int8_t   stations_bin_for(colour_t c);                // -1 = none
void     stations_commit(void);                       // write EEPROM, layout version + 1
void     stations_revert(void);                       // reload EEPROM (or CAD defaults)
void     stations_factory(void);                      // CAD defaults (still needs commit)
bool     stations_dirty(void);
uint8_t  stations_layout_version(void);               // +1 per commit (telemetry: sim re-reads)

// would this station be OK? self = its own index (skipped by the spacing check), -1 = new
st_err_t stations_validate(const pose_t *p, int16_t above_dz, uint8_t type, int8_t self);
