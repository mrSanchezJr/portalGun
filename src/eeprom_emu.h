#pragma once
#include <stdint.h>

void eeprom_init(void);
bool eeprom_read(uint16_t addr, uint16_t &data);
void eeprom_write(uint16_t addr, uint16_t data);
