// LAYER 2 — DEVICES · storage (EEPROM records with a magic number + CRC-16)
// A record is [magic][data][crc]. A bad magic or CRC (blank chip, layout change,
// half-finished write) makes storage_load() return false: the caller keeps its defaults.
#pragma once
#include "../config.h"

bool storage_load(uint16_t addr, uint16_t magic, void *data, uint16_t len);   // data untouched on failure
void storage_save(uint16_t addr, uint16_t magic, const void *data, uint16_t len);
