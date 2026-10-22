// Adapted from the ExpressLRS Backpack project (lib/config/config.cpp),
// trimmed to the Timer Backpack config struct only.
// https://github.com/ExpressLRS/Backpack — GPL-3.0-only
// Copyright (c) ExpressLRS contributors
#include "bp_config.h"
#include <string.h>

TimerBackpackConfig bpConfig;

void
TimerBackpackConfig::Load()
{
    m_eeprom->Get(0, m_config);
    m_modified = false;

    if (m_config.version != (uint32_t)(TIMER_BACKPACK_CONFIG_VERSION | TIMER_BACKPACK_CONFIG_MAGIC))
    {
        SetDefaults();
    }
}

void
TimerBackpackConfig::Commit()
{
    if (!m_modified) return;
    m_eeprom->Put(0, m_config);
    m_eeprom->Commit();
    m_modified = false;
}

void
TimerBackpackConfig::SetStorageProvider(ELRS_EEPROM *eeprom)
{
    if (eeprom) m_eeprom = eeprom;
}

void
TimerBackpackConfig::SetDefaults()
{
    m_config.version = TIMER_BACKPACK_CONFIG_VERSION | TIMER_BACKPACK_CONFIG_MAGIC;
    m_config.bootCount = 0;
    memset(m_config.address, 0, 6);
    m_config.phrase[0] = 0;
    m_modified = true;
    Commit();
}

void
TimerBackpackConfig::SetGroupAddress(const uint8_t address[6])
{
    memcpy(m_config.address, address, 6);
    m_modified = true;
}

void
TimerBackpackConfig::SetPhrase(const char *phrase)
{
    strncpy(m_config.phrase, phrase, sizeof(m_config.phrase) - 1);
    m_config.phrase[sizeof(m_config.phrase) - 1] = 0;
    m_modified = true;
}

void
TimerBackpackConfig::SetBootCount(uint8_t count)
{
    m_config.bootCount = count;
    m_modified = true;
}
