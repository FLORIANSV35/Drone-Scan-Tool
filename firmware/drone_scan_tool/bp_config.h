// Adapted from the ExpressLRS Backpack project (lib/config/config.h),
// trimmed to the Timer Backpack config struct only.
// https://github.com/ExpressLRS/Backpack — GPL-3.0-only
// Copyright (c) ExpressLRS contributors
//
// Named bp_config.h (not config.h) to avoid clashing with this sketch's
// own config.h (pins, scan range, sniffer settings).
#pragma once

#include "elrs_eeprom.h"

#define TIMER_BACKPACK_CONFIG_MAGIC   (0b11U << 30)
#define TIMER_BACKPACK_CONFIG_VERSION 3

typedef struct {
    uint32_t    version;
    uint8_t     bootCount;
    uint8_t     address[6];   // group address (UID), derived from the binding phrase
} timer_backpack_config_t;

class TimerBackpackConfig
{
public:
    void Load();
    void Commit();

    bool     IsModified() const { return m_modified; }
    uint8_t  GetBootCount() { return m_config.bootCount; }
    uint8_t *GetGroupAddress() { return m_config.address; }

    void SetStorageProvider(ELRS_EEPROM *eeprom);
    void SetDefaults();
    void SetBootCount(uint8_t count);
    void SetGroupAddress(const uint8_t address[6]);

private:
    timer_backpack_config_t m_config;
    ELRS_EEPROM             *m_eeprom;
    bool                    m_modified;
};

extern TimerBackpackConfig bpConfig;
