// LAYER 2 — DEVICES · sensors (limit switches, E-stop, IR, colour)
#include "sensors.h"
#include "../1_hal/hal.h"

// ── debounced digital input ──
typedef struct { uint8_t pin; bool stable, last_raw; uint32_t t_change; uint16_t on_ms, off_ms; } input_t;

static input_t in_limit[N_STEPPERS] = {
  {PIN_LIMIT_J1, 0, 0, 0, LIMIT_DEBOUNCE_MS, LIMIT_DEBOUNCE_MS},
  {PIN_LIMIT_J2, 0, 0, 0, LIMIT_DEBOUNCE_MS, LIMIT_DEBOUNCE_MS},
  {PIN_LIMIT_J3, 0, 0, 0, LIMIT_DEBOUNCE_MS, LIMIT_DEBOUNCE_MS}};
static input_t in_estop = {PIN_ESTOP, 0, 0, 0, ESTOP_ON_MS, ESTOP_OFF_MS};
static input_t in_ir    = {PIN_IR,    0, 0, 0, IR_STABLE_MS, IR_CLEAR_MS};

static void debounce(input_t *in, bool raw, uint32_t now) {
  if (raw != in->last_raw) { in->last_raw = raw; in->t_change = now; }
  uint16_t need = raw ? in->on_ms : in->off_ms;
  if (raw != in->stable && (now - in->t_change) >= need) in->stable = raw;
}

void sensors_init(void) {
  for (uint8_t i = 0; i < N_STEPPERS; i++) hal_pin_input_pullup(in_limit[i].pin);
  hal_pin_input_pullup(in_estop.pin);
  hal_pin_input_pullup(in_ir.pin);
  hal_pin_output(PIN_TCS_LED); hal_pin_write(PIN_TCS_LED, true);   // light under the cube
}

void sensors_update(uint32_t now) {
  for (uint8_t i = 0; i < N_STEPPERS; i++)
    debounce(&in_limit[i], hal_pin_read(in_limit[i].pin) == 1, now); // NC: HIGH = pressed / broken
  debounce(&in_estop, hal_pin_read(in_estop.pin) == 0, now);         // no motor power
  debounce(&in_ir,    hal_pin_read(in_ir.pin)    == 0, now);         // NPN: LOW = object
}

bool limit_pressed(uint8_t axis) { return in_limit[axis].stable; }
bool estop_active(void)          { return in_estop.stable; }
bool ir_object(void)             { return in_ir.stable; }

// ── TCS34725 colour sensor ──
#define TCS_CMD      0x80
#define TCS_AUTOINC  0x20
#define TCS_ENABLE   0x00
#define TCS_ATIME    0x01
#define TCS_CONTROL  0x0F
#define TCS_ID       0x12
#define TCS_STATUS   0x13
#define TCS_CDATAL   0x14
#define TCS_PON      0x01
#define TCS_AEN      0x02
#define TCS_AVALID   0x01
#define TCS_ID_34725 0x44
#define TCS_ID_34727 0x4D
static uint32_t t_colour;

static bool tcs_write(uint8_t reg, uint8_t val) {
  uint8_t m[2] = {(uint8_t)(TCS_CMD | reg), val};
  return hal_i2c_write(I2C_TCS34725, m, 2);
}

bool colour_init(void) {
  uint8_t id = 0;
  if (!hal_i2c_read(I2C_TCS34725, TCS_CMD | TCS_ID, &id, 1)) return false;
  if (id != TCS_ID_34725 && id != TCS_ID_34727) return false;
  return tcs_write(TCS_ATIME, TCS_ATIME_VAL) && tcs_write(TCS_CONTROL, TCS_GAIN_VAL) &&
         tcs_write(TCS_ENABLE, TCS_PON) && tcs_write(TCS_ENABLE, TCS_PON | TCS_AEN);  // continuous
}

void colour_start(void) { t_colour = hal_millis(); }

bool colour_ready(void) {
  if (hal_millis() - t_colour < COLOUR_INTEG_MS) return false;   // wait one full integration
  uint8_t st = 0;
  return hal_i2c_read(I2C_TCS34725, TCS_CMD | TCS_STATUS, &st, 1) && (st & TCS_AVALID);
}

rgbc_t colour_get(void) {
  uint8_t d[8] = {0};
  hal_i2c_read(I2C_TCS34725, TCS_CMD | TCS_AUTOINC | TCS_CDATAL, d, 8);
  rgbc_t c = { (uint16_t)(d[2] | d[3] << 8), (uint16_t)(d[4] | d[5] << 8),
               (uint16_t)(d[6] | d[7] << 8), (uint16_t)(d[0] | d[1] << 8) };
  return c;
}
