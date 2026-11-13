// main.cpp — the whole program: start everything, then loop forever. Nothing here blocks.
#include <Arduino.h>
extern "C" {
#include "1_hal/hal.h"
#include "6_app/fsm.h"
#include "6_app/console.h"
}

void setup() {
  hal_init();        // serial, I²C, watchdog
  fsm_init();        // sensors, motors, saved poses → state BOOT
}

void loop() {
  uint32_t now = hal_millis();
  fsm_tick(now);       // what the robot does
  console_update(now); // commands in, replies/events/telemetry out
  hal_watchdog_kick();
}
