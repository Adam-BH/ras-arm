// LAYER 1 — HAL (hardware abstraction layer)
// The ONLY place that knows we are on an Arduino. Everything above calls these
// functions. Swap the implementation (hal_avr.cpp ↔ test/hal_fake.c) and nothing else changes.
#pragma once
#include "../config.h"

void     hal_init(void);
uint32_t hal_millis(void);
void     hal_watchdog_kick(void);          // call every loop(); reset after WATCHDOG_MS without it

// interrupts off/on around multi-byte data shared with the stepper ISR
uint8_t  hal_irq_save(void);
void     hal_irq_restore(uint8_t state);

// digital pins
void hal_pin_output(uint8_t pin);
void hal_pin_input_pullup(uint8_t pin);
void hal_pin_write(uint8_t pin, bool level);
bool hal_pin_read(uint8_t pin);

// I²C (PCA9685, TCS34725) — never hangs: bus timeout → false
bool hal_i2c_write(uint8_t addr, const uint8_t *data, uint8_t len);
bool hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);

// step generation: start a timer that calls isr() hz times per second,
// and a FAST pulse on several STEP pins at once (bit0 = J1, bit1 = J2, bit2 = J3)
void hal_step_timer_start(uint16_t hz, void (*isr)(void));
void hal_set_dirs(uint8_t dir_mask);
void hal_step_pulse(uint8_t step_mask);

// non-volatile memory (taught poses, colour calibration)
void hal_eeprom_read(uint16_t addr, void *data, uint16_t len);
void hal_eeprom_write(uint16_t addr, const void *data, uint16_t len);

// serial link (console, teach UI, telemetry)
int  hal_serial_read(void);              // -1 if nothing
void hal_serial_write(const char *text);
