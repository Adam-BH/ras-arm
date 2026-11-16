// LAYER 4 — MOTION · stations
#include "stations.h"
#include "../3_joints/joints.h"
#include "../2_devices/storage.h"
#include "../flash.h"
#include <math.h>
#include <string.h>

// CAD default layout (config.h numbers), in flash
#define T(v) ((int16_t)((v) * 10.0f))
static const station_t CAD[] FLASH = {
  {"HOME",  STN_PARK, T(CELL_HOME_X),   0,              T(CELL_HOME_Z), 0,                 STATION_ABOVE_DZ},
  {"CUBE",  STN_PICK, T(CELL_CRADLE_X), 0,              T(CELL_GRIP_Z), 0,                 STATION_ABOVE_DZ},
  {"BIN_A", STN_BIN,  0,                T(CELL_BIN_Y),  T(CELL_DROP_Z), T(CELL_BIN_ROLL),  STATION_ABOVE_DZ},
  {"BIN_B", STN_BIN,  0,               -T(CELL_BIN_Y),  T(CELL_DROP_Z), -T(CELL_BIN_ROLL), STATION_ABOVE_DZ},
};
enum { CAD_BIN_A = 2, CAD_BIN_B = 3, CAD_N = sizeof CAD / sizeof CAD[0] };   // RED → BIN_A, BLUE → BIN_B

// the RAM table IS the EEPROM record: [magic][ver count layout stations map][crc]
static struct {
  uint8_t   ver, count, layout;
  station_t st[MAX_STATIONS];
  int8_t    map[N_COLOURS];        // colour_t → station index, -1 = none
} rec;
static bool dirty;

static float   tenth(int16_t v) { return v * 0.1f; }
static int16_t to10(float v)    { return (int16_t)(v * 10.0f + (v < 0 ? -0.5f : 0.5f)); }
static float   above_z(float z, int16_t dz) { float a = z + dz; return a > SAFE_Z ? a : SAFE_Z; }

uint8_t          stations_count(void)          { return rec.count; }
const station_t *stations_get(uint8_t i)       { return &rec.st[i]; }
bool             stations_dirty(void)          { return dirty; }
uint8_t          stations_layout_version(void) { return rec.layout; }
int8_t           stations_bin_for(colour_t c)  { return c < N_COLOURS ? rec.map[c] : -1; }

pose_t stations_grip(uint8_t i) {
  const station_t *s = &rec.st[i];
  pose_t p = {tenth(s->x), tenth(s->y), tenth(s->z), tenth(s->roll)};
  return p;
}
pose_t stations_above(uint8_t i) {                 // park poses have no approach: GO goes straight there
  pose_t p = stations_grip(i);
  if (rec.st[i].type != STN_PARK) p.z = above_z(p.z, rec.st[i].above_dz);
  return p;
}

int8_t stations_find(const char *name) {
  for (uint8_t i = 0; i < rec.count; i++) if (!strncmp(rec.st[i].name, name, STATION_NAME_LEN)) return (int8_t)i;
  return -1;
}
int8_t stations_find_type(uint8_t type, uint8_t nth) {
  for (uint8_t i = 0; i < rec.count; i++) if (rec.st[i].type == type && nth-- == 0) return (int8_t)i;
  return -1;
}

// ── load / save ──
void stations_factory(void) {
  rec.ver = STATIONS_REC_VERSION; rec.count = CAD_N;
  FMEMCPY(rec.st, CAD, sizeof CAD);
  memset(rec.map, -1, sizeof rec.map);
  rec.map[COL_RED] = CAD_BIN_A; rec.map[COL_BLUE] = CAD_BIN_B;
  dirty = true;
}
static bool sane(void) {                          // a good CRC from an older/odd layout is still refused
  if (rec.ver != STATIONS_REC_VERSION || rec.count < 2 || rec.count > MAX_STATIONS ||
      rec.st[STATION_HOME].type != STN_PARK || stations_find_type(STN_PICK, 0) < 0) return false;
  for (uint8_t c = 0; c < N_COLOURS; c++) if (rec.map[c] >= (int8_t)rec.count) return false;
  return true;
}
void stations_revert(void) {
  if (!storage_load(EE_ADDR_STATIONS, EE_MAGIC_STATIONS, &rec, sizeof rec) || !sane()) { stations_factory(); rec.layout = 0; }
  dirty = false;
}
void stations_init(void)   { stations_revert(); }
void stations_commit(void) { rec.layout++; storage_save(EE_ADDR_STATIONS, EE_MAGIC_STATIONS, &rec, sizeof rec); dirty = false; }

// ── validation ──
static st_err_t check(const pose_t *p, uint8_t type) {
  if (p->z < Z_MIN_TABLE || sqrtf(p->x * p->x + p->y * p->y) < R_MIN_BASE) return STE_KEEPOUT;
  angles_t q = {{0}};
  ik_result_t k = ik_solve(p, &q);
  if (k == IK_UNREACHABLE) { float c = ik_cos3(p); return c > IK_MAX_COS3 && c <= 1.0f ? STE_SINGULAR : STE_UNREACHABLE; }
  if (k == IK_LIMIT) return STE_LIMIT;
  if (type != STN_PARK && (q.j[J1] < J1_WORK_MIN || q.j[J1] > J1_WORK_MAX)) return STE_LIMIT;
  return STE_OK;
}

st_err_t stations_validate(const pose_t *p, int16_t dz, uint8_t type, int8_t self) {
  if (dz < 0 || (type == STN_BIN && p->z < BIN_MIN_DROP_Z)) return STE_KEEPOUT;
  float top = type == STN_PARK ? p->z : above_z(p->z, dz);
  pose_t a = *p;
  for (;;) {                                      // grip → approach (crosses SAFE_Z): the line the arm will run
    st_err_t e = check(&a, type);
    if (e) return e;
    if (a.z >= top) break;
    a.z = a.z + LINE_STEP_MM < top ? a.z + LINE_STEP_MM : top;
  }
  if (type == STN_PARK) return STE_OK;            // park poses are in the air: no footprint
  for (uint8_t i = 0; i < rec.count; i++) {
    if ((int8_t)i == self || rec.st[i].type == STN_PARK) continue;
    float dx = p->x - tenth(rec.st[i].x), dy = p->y - tenth(rec.st[i].y);
    if (dx * dx + dy * dy < MIN_STATION_GAP * MIN_STATION_GAP) return STE_TOO_CLOSE;
  }
  return STE_OK;
}

// ── edits (RAM only) ──
static bool name_ok(const char *n) {
  uint8_t k = 0;
  for (; n[k]; k++) {
    char c = n[k];
    if (k >= STATION_NAME_LEN - 1 || !((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_')) return false;
  }
  return k > 0;
}
static st_err_t put(uint8_t i, const pose_t *p, int16_t dz, int8_t self) {
  pose_t q = {to10(p->x) * 0.1f, to10(p->y) * 0.1f, to10(p->z) * 0.1f, to10(p->roll) * 0.1f};   // what gets stored
  st_err_t e = stations_validate(&q, dz, rec.st[i].type, self);
  if (e) return e;
  station_t *s = &rec.st[i];
  s->x = to10(p->x); s->y = to10(p->y); s->z = to10(p->z); s->roll = to10(p->roll); s->above_dz = dz;
  dirty = true;
  return STE_OK;
}

st_err_t stations_add(const char *name, uint8_t type, const pose_t *p, int16_t dz) {
  if (!name_ok(name) || type > STN_BIN || stations_find(name) >= 0) return STE_NAME;
  if (rec.count >= MAX_STATIONS || (type == STN_PICK && stations_find_type(STN_PICK, 0) >= 0)) return STE_FULL;
  station_t *s = &rec.st[rec.count];              // fill the free slot, count it only if valid
  memset(s, 0, sizeof *s); strcpy(s->name, name); s->type = type;
  st_err_t e = put(rec.count, p, dz, -1);
  if (!e) rec.count++;
  return e;
}

st_err_t stations_set(uint8_t i, const pose_t *p, int16_t dz) { return put(i, p, dz, (int8_t)i); }

st_err_t stations_delete(uint8_t i) {
  if (i == STATION_HOME || (int8_t)i == stations_find_type(STN_PICK, 0)) return STE_PROTECTED;
  memmove(&rec.st[i], &rec.st[i + 1], (rec.count - i - 1) * sizeof(station_t));
  rec.count--;
  for (uint8_t c = 0; c < N_COLOURS; c++) {       // keep the map pointing at the same stations
    if (rec.map[c] == (int8_t)i) rec.map[c] = -1;
    else if (rec.map[c] > (int8_t)i) rec.map[c]--;
  }
  dirty = true;
  return STE_OK;
}

st_err_t stations_map(colour_t c, int8_t i) {
  if (c == COL_EMPTY || c >= N_COLOURS || (i >= 0 && rec.st[i].type != STN_BIN)) return STE_NAME;
  rec.map[c] = i; dirty = true;
  return STE_OK;
}
