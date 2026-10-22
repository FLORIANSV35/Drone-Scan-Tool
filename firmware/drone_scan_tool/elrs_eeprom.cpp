// Adapted from the ExpressLRS Backpack project (lib/EEPROM)
// https://github.com/ExpressLRS/Backpack — GPL-3.0-only
// Copyright (c) ExpressLRS contributors
#include "elrs_eeprom.h"
#include <EEPROM.h>

void
ELRS_EEPROM::Begin()
{
    EEPROM.begin(RESERVED_EEPROM_SIZE);
}

uint8_t
ELRS_EEPROM::ReadByte(const uint32_t address)
{
    if (address >= RESERVED_EEPROM_SIZE) return 0;
    return EEPROM.read(address);
}

void
ELRS_EEPROM::WriteByte(const uint32_t address, const uint8_t value)
{
    if (address >= RESERVED_EEPROM_SIZE) return;
    EEPROM.write(address, value);
}

void
ELRS_EEPROM::Commit()
{
    EEPROM.commit();
}
