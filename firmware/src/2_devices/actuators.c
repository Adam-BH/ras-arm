// LAYER 2 — DEVICES · actuators (steppers + servos + status LED)
#include "actuators.h"
#include "../1_hal/hal.h"

// ════════════════════════ STEPPERS ════════════════════════
// One timer interrupt drives all 3 axes with a Bresenham "DDA": the axis with the
// most steps (master) sets the pace, the others step proportionally → they all
// start and finish together. The main loop only changes 'rate' (trapezoid speed ramp).

typedef enum { ST_IDLE, ST_MOVE, ST_RUN } st_mode_t;
static const int8_t DIR_SIGN[N_STEPPERS] = {DIR_SIGN_J1, DIR_SIGN_J2, DIR_SIGN_J3};
static volatile st_mode_t mode = ST_IDLE;
static volatile int32_t  pos[N_STEPPERS];
static volatile uint32_t delta[N_STEPPERS], err[N_STEPPERS], master, done;
static volatile uint8_t  dir_mask;                 // logical direction (bit set = +)
static volatile uint32_t acc, rate_q16;            // rate = master steps per tick × 65536
static uint32_t t_start, t_total;
static float    v_peak;                            // master steps per second

static void step_isr(void) {
  if (mode == ST_IDLE) return;
  acc += rate_q16;
  if (acc < 65536UL) return;
  acc -= 65536UL;
  uint8_t pulse = 0;
  for (uint8_t i = 0; i < N_STEPPERS; i++) {
    err[i] += delta[i];
    if (err[i] >= master) {
      err[i] -= master;
      pulse |= 1 << i;
      pos[i] += (dir_mask & (1 << i)) ? 1 : -1;
    }
  }
  if (pulse) hal_step_pulse(pulse);
  if (mode == ST_MOVE && ++done >= master) mode = ST_IDLE;
}

static void set_dirs(uint8_t logical) {             // apply the per-motor wiring sign
  uint8_t hw = logical;
  for (uint8_t i = 0; i < N_STEPPERS; i++) if (DIR_SIGN[i] < 0) hw ^= 1 << i;
  dir_mask = logical; hal_set_dirs(hw);
}

void steppers_init(void) {
  static const uint8_t pins[] = {PIN_STEP_J1, PIN_STEP_J2, PIN_STEP_J3, PIN_DIR_J1, PIN_DIR_J2, PIN_DIR_J3, PIN_EN};
  for (uint8_t i = 0; i < sizeof pins; i++) hal_pin_output(pins[i]);
  mode = ST_IDLE;
  steppers_enable(false);
  hal_step_timer_start(STEP_ISR_HZ, step_isr);
}

void steppers_enable(bool on) { hal_pin_write(PIN_EN, !on); }   // active LOW

void steppers_move_to(const int32_t target[N_STEPPERS], uint32_t duration_ms) {
  mode = ST_IDLE;                                    // ISR goes quiet while we set up
  uint32_t m = 0; uint8_t dm = 0;
  for (uint8_t i = 0; i < N_STEPPERS; i++) {
    int32_t d = target[i] - stepper_pos(i);
    if (d > 0) dm |= 1 << i;
    delta[i] = d > 0 ? (uint32_t)d : (uint32_t)-d;
    if (delta[i] > m) m = delta[i];
  }
  if (m == 0) return;
  for (uint8_t i = 0; i < N_STEPPERS; i++) err[i] = m / 2;
  set_dirs(dm);
  master = m; done = 0; acc = 0;
  if (duration_ms < MIN_MOVE_MS) duration_ms = MIN_MOVE_MS;
  // trapezoid: average speed = peak × (1 − RAMP_FRACTION)
  v_peak  = (float)m / ((1.0f - RAMP_FRACTION) * duration_ms / 1000.0f);
  t_start = hal_millis(); t_total = duration_ms;
  rate_q16 = (uint32_t)(RAMP_MIN_RATE * v_peak / STEP_ISR_HZ * 65536.0f);
  mode = ST_MOVE;
}

void stepper_run(uint8_t axis, int8_t dir, float steps_per_s) {
  mode = ST_IDLE;
  for (uint8_t i = 0; i < N_STEPPERS; i++) { delta[i] = (i == axis); err[i] = 0; }
  set_dirs(dir > 0 ? (uint8_t)(1 << axis) : 0);
  master = 1; acc = 0;
  float r = steps_per_s / STEP_ISR_HZ; if (r > 1.0f) r = 1.0f;
  rate_q16 = (uint32_t)(r * 65536.0f);
  mode = ST_RUN;
}

void steppers_stop(void) { mode = ST_IDLE; }
bool steppers_busy(void) { return mode != ST_IDLE; }

int32_t stepper_pos(uint8_t a) {                     // 32-bit read must not be torn by the ISR
  uint8_t s = hal_irq_save(); int32_t p = pos[a]; hal_irq_restore(s); return p;
}
void stepper_set_pos(uint8_t a, int32_t steps) {
  uint8_t s = hal_irq_save(); pos[a] = steps; hal_irq_restore(s);
}

void steppers_update(uint32_t now) {
  if (mode != ST_MOVE) return;
  float t = (float)(now - t_start) / t_total;         // 0 … 1
  float f = 1.0f;
  if (t < RAMP_FRACTION)             f = t / RAMP_FRACTION;
  else if (t > 1.0f - RAMP_FRACTION) f = (1.0f - t) / RAMP_FRACTION;
  if (f < RAMP_MIN_RATE) f = RAMP_MIN_RATE;           // never stall before the last step
  float r = v_peak * f / STEP_ISR_HZ;                 // steps per tick
  if (r > 1.0f) r = 1.0f;
  uint32_t q = (uint32_t)(r * 65536.0f);
  uint8_t s = hal_irq_save(); rate_q16 = q; hal_irq_restore(s);
}

// ════════════════════════ SERVOS (PCA9685) ════════════════════════
#define PCA_MODE1    0x00
#define PCA_PRESCALE 0xFE
#define PCA_LED0     0x06
#define PCA_SLEEP    0x10
#define PCA_AI       0x20
#define PCA_RESTART  0x80
#define SERVO_PERIOD_US (1000000UL / SERVO_FRAME_HZ)

bool servos_init(void) {
  uint8_t sleep[]   = {PCA_MODE1, PCA_SLEEP};          // sleep (needed to set the prescaler)
  uint8_t pre[]     = {PCA_PRESCALE, (uint8_t)((PCA9685_OSC_HZ + 2048UL * SERVO_FRAME_HZ) / (4096UL * SERVO_FRAME_HZ) - 1)};
  uint8_t wake[]    = {PCA_MODE1, PCA_AI};             // auto-increment, awake
  uint8_t restart[] = {PCA_MODE1, PCA_AI | PCA_RESTART};
  return hal_i2c_write(I2C_PCA9685, sleep, 2) && hal_i2c_write(I2C_PCA9685, pre, 2) &&
         hal_i2c_write(I2C_PCA9685, wake, 2)  && hal_i2c_write(I2C_PCA9685, restart, 2);
}

void servo_write_us(uint8_t ch, uint16_t us) {
  uint16_t off = (uint16_t)((uint32_t)us * 4096UL / SERVO_PERIOD_US);
  uint8_t msg[5] = {(uint8_t)(PCA_LED0 + 4 * ch), 0, 0, (uint8_t)(off & 0xFF), (uint8_t)(off >> 8)};
  hal_i2c_write(I2C_PCA9685, msg, 5);
}

// ════════════════════════ STATUS LED ════════════════════════
void status_led_update(uint32_t now, uint8_t pattern) {
  static bool init;
  if (!init) { hal_pin_output(PIN_LED); init = true; }
  bool on;
  if (pattern == LED_SOLID) on = true;
  else if (pattern == 0)    on = (now / LED_IDLE_MS) & 1;
  else {
    uint32_t cycle = 2UL * LED_BLINK_MS * pattern + LED_PAUSE_MS, t = now % cycle;
    on = t < 2UL * LED_BLINK_MS * pattern && (t / LED_BLINK_MS) % 2 == 0;
  }
  hal_pin_write(PIN_LED, on);
}
