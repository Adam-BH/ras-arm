// LAYER 1 — HAL for the real Arduino UNO (the only C++ / Arduino file).
#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
extern "C" {
#include "hal.h"
}

static void (*step_isr)(void) = 0;

extern "C" void hal_init(void) {
  MCUSR = 0; wdt_disable();                 // a watchdog reset must not loop
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true); // never hang on a bad I²C wire
  wdt_enable(WDTO_250MS);                    // WATCHDOG_MS
}

extern "C" uint32_t hal_millis(void)                 { return millis(); }
extern "C" void hal_watchdog_kick(void)              { wdt_reset(); }
extern "C" uint8_t hal_irq_save(void)                { uint8_t s = SREG; cli(); return s; }
extern "C" void hal_irq_restore(uint8_t s)           { SREG = s; }
extern "C" void hal_pin_output(uint8_t p)            { pinMode(p, OUTPUT); }
extern "C" void hal_pin_input_pullup(uint8_t p)      { pinMode(p, INPUT_PULLUP); }
extern "C" void hal_pin_write(uint8_t p, bool v)     { digitalWrite(p, v); }
extern "C" bool hal_pin_read(uint8_t p)              { return digitalRead(p); }

extern "C" bool hal_i2c_write(uint8_t a, const uint8_t *d, uint8_t n) {
  Wire.beginTransmission(a); Wire.write(d, n); return Wire.endTransmission() == 0;
}
extern "C" bool hal_i2c_read(uint8_t a, uint8_t reg, uint8_t *d, uint8_t n) {
  Wire.beginTransmission(a); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(a, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) d[i] = Wire.read();
  return true;
}

// Timer1 in CTC mode → calls the stepper ISR at 'hz'
extern "C" void hal_step_timer_start(uint16_t hz, void (*isr)(void)) {
  step_isr = isr;
  noInterrupts();
  TCCR1A = 0; TCCR1B = (1 << WGM12) | (1 << CS11);     // CTC, prescaler 8 → 2 MHz
  OCR1A  = (F_CPU / 8 / hz) - 1;
  TIMSK1 = (1 << OCIE1A);
  interrupts();
}
ISR(TIMER1_COMPA_vect) { if (step_isr) step_isr(); }

// STEP = PD2..PD4, DIR = PD5..PD7 → direct port writes (fast, inside the ISR)
extern "C" void hal_set_dirs(uint8_t m)  {
  uint8_t s = SREG; cli(); PORTD = (PORTD & 0x1F) | ((m & 0x07) << 5); SREG = s;
}
extern "C" void hal_step_pulse(uint8_t m) {
  PORTD |= (m & 0x07) << 2;
  __builtin_avr_delay_cycles(32);                     // 2 µs high (TMC2209 needs > 100 ns)
  PORTD &= ~((m & 0x07) << 2);
}

extern "C" void hal_eeprom_read(uint16_t a, void *d, uint16_t n) {
  for (uint16_t i = 0; i < n; i++) ((uint8_t *)d)[i] = EEPROM.read(a + i);
}
// each changed byte takes ~3.4 ms: kick the watchdog so a pose save can't reset the board
extern "C" void hal_eeprom_write(uint16_t a, const void *d, uint16_t n) {
  for (uint16_t i = 0; i < n; i++) { EEPROM.update(a + i, ((const uint8_t *)d)[i]); wdt_reset(); }
}

extern "C" int  hal_serial_read(void)               { return Serial.available() ? Serial.read() : -1; }
extern "C" void hal_serial_write(const char *s)     { Serial.write(s); }
#endif
