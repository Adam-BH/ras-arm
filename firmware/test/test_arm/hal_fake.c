// test/hal_fake.c — a FAKE robot that implements hal.h so everything runs on a PC.
// It counts the real step pulses (like a physical motor), presses the limit switches
// when a joint reaches them, and lets the test place cubes, cut power, break wires, type commands.
#include "../../src/1_hal/hal.h"
#include <string.h>
#include <stdio.h>

static const float SPD_F[N_STEPPERS] = {SPD_J1, SPD_J2, SPD_J3};

uint32_t fake_now;
int32_t  fake_true_steps[N_STEPPERS];          // where the motors REALLY are
bool     fake_power_ok = true, fake_ir_blocked = false;
bool     fake_limit_broken[N_STEPPERS];        // wire cut: NC input reads HIGH = "pressed" forever
bool     fake_limit_dead[N_STEPPERS];          // switch never closes
rgbc_t   fake_rgbc = {60, 65, 70, 200};        // what the colour sensor sees
uint16_t fake_servo_us[16];
static uint8_t  dirs;
static void   (*isr)(void);
static uint16_t isr_hz;
static uint8_t  eeprom[1024];
static char     in_buf[1024]; static int in_r, in_w;
char            fake_log[8192]; int fake_log_n;  // every non-telemetry line sent
char            fake_last_tel[128];              // the latest telemetry line

void fake_wipe_eeprom(void)      { memset(eeprom, 0xFF, sizeof eeprom); }
uint8_t *fake_eeprom(void)       { return eeprom; }
void fake_reset(float j1, float j2, float j3) {           // arm left in an unknown pose
  float a[3] = {j1, j2, j3};
  for (int i = 0; i < N_STEPPERS; i++) {
    fake_true_steps[i] = (int32_t)(a[i] * SPD_F[i]);
    fake_limit_broken[i] = fake_limit_dead[i] = false;
  }
  fake_power_ok = true; fake_ir_blocked = false;
  in_r = in_w = 0;
}
void fake_clear_log(void)     { fake_log_n = 0; fake_log[0] = 0; }
void fake_type(const char *s) { while (*s) { in_buf[in_w++ % sizeof in_buf] = *s++; } }
void fake_tick_1ms(void)      { fake_now++; for (int k = 0; k < isr_hz / 1000; k++) if (isr) isr(); }
float fake_true_deg(int i)    { return fake_true_steps[i] / SPD_F[i]; }

static bool switch_pressed(int i) {
  if (fake_limit_broken[i]) return true;
  if (fake_limit_dead[i]) return false;
  float a = fake_true_deg(i);
  return HOME_DIR[i] < 0 ? a <= HOME_SWITCH_DEG[i] : a >= HOME_SWITCH_DEG[i];
}

void     hal_init(void)                    {}
uint32_t hal_millis(void)                  { return fake_now; }
void     hal_watchdog_kick(void)           {}
uint8_t  hal_irq_save(void)                { return 0; }
void     hal_irq_restore(uint8_t s)        { (void)s; }
void     hal_pin_output(uint8_t p)         { (void)p; }
void     hal_pin_input_pullup(uint8_t p)   { (void)p; }
void     hal_pin_write(uint8_t p, bool v)  { (void)p; (void)v; }
bool     hal_pin_read(uint8_t p) {
  if (p == PIN_LIMIT_J1) return switch_pressed(0);         // NC wiring: pressed = HIGH
  if (p == PIN_LIMIT_J2) return switch_pressed(1);
  if (p == PIN_LIMIT_J3) return switch_pressed(2);
  if (p == PIN_ESTOP)    return fake_power_ok;
  if (p == PIN_IR)       return !fake_ir_blocked;          // LOW = object
  return 0;
}

bool hal_i2c_write(uint8_t a, const uint8_t *d, uint8_t n) {
  if (a == I2C_PCA9685 && n == 5 && d[0] >= 0x06) fake_servo_us[(d[0] - 6) / 4] = (uint16_t)((d[3] | d[4] << 8) * 20000UL / 4096);
  return true;
}
bool hal_i2c_read(uint8_t a, uint8_t reg, uint8_t *d, uint8_t n) {
  if (a != I2C_TCS34725) return false;
  reg &= 0x1F;
  if (reg == 0x12) { d[0] = 0x44; return true; }           // ID
  if (reg == 0x13) { d[0] = 0x01; return true; }           // AVALID
  if (reg == 0x14 && n == 8) {
    uint16_t v[4] = {fake_rgbc.c, fake_rgbc.r, fake_rgbc.g, fake_rgbc.b};
    for (int i = 0; i < 4; i++) { d[2 * i] = v[i] & 0xFF; d[2 * i + 1] = v[i] >> 8; }
    return true;
  }
  return true;
}

void hal_step_timer_start(uint16_t hz, void (*f)(void)) { isr_hz = hz; isr = f; }
void hal_set_dirs(uint8_t m) { dirs = m; }
void hal_step_pulse(uint8_t m) {
  if (!fake_power_ok) return;                              // no power: motor doesn't move
  for (int i = 0; i < N_STEPPERS; i++) if (m & (1 << i)) fake_true_steps[i] += (dirs & (1 << i)) ? 1 : -1;
}

void hal_eeprom_read(uint16_t a, void *d, uint16_t n)        { memcpy(d, eeprom + a, n); }
void hal_eeprom_write(uint16_t a, const void *d, uint16_t n) { memcpy(eeprom + a, d, n); }

int  hal_serial_read(void) { return in_r < in_w ? (unsigned char)in_buf[in_r++ % sizeof in_buf] : -1; }
void hal_serial_write(const char *s) {                     // the console sends one line per write
  size_t L = strlen(s);
  if (s[0] == 'T' && s[1] == ',') { snprintf(fake_last_tel, sizeof fake_last_tel, "%s", s); return; }
  if (fake_log_n + L >= sizeof fake_log - 1) fake_clear_log();
  memcpy(fake_log + fake_log_n, s, L); fake_log_n += (int)L; fake_log[fake_log_n] = 0;
}
