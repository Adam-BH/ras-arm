// LAYER 2 — DEVICES · storage
#include "storage.h"
#include "../1_hal/hal.h"

#define CRC_INIT 0xFFFF
#define CRC_POLY 0xA001     // CRC-16/MODBUS

static uint16_t crc_byte(uint16_t c, uint8_t b) {
  c ^= b;
  for (uint8_t i = 0; i < 8; i++) c = (c & 1) ? (c >> 1) ^ CRC_POLY : c >> 1;
  return c;
}

bool storage_load(uint16_t addr, uint16_t magic, void *data, uint16_t len) {
  uint16_t m, stored, c = CRC_INIT;
  hal_eeprom_read(addr, &m, 2);
  if (m != magic) return false;
  for (uint16_t i = 0; i < len; i++) {        // check the CRC byte by byte first: no RAM copy,
    uint8_t b; hal_eeprom_read(addr + 2 + i, &b, 1);   // and the live data stays intact if bad
    c = crc_byte(c, b);
  }
  hal_eeprom_read(addr + 2 + len, &stored, 2);
  if (stored != c) return false;
  hal_eeprom_read(addr + 2, data, len);
  return true;
}

void storage_save(uint16_t addr, uint16_t magic, const void *data, uint16_t len) {
  uint16_t c = CRC_INIT;
  for (uint16_t i = 0; i < len; i++) c = crc_byte(c, ((const uint8_t *)data)[i]);
  hal_eeprom_write(addr, &magic, 2);
  hal_eeprom_write(addr + 2, data, len);
  hal_eeprom_write(addr + 2 + len, &c, 2);
}
