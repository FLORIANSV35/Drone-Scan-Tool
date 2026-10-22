// Adapted from the ExpressLRS Backpack project (lib/EEPROM)
// https://github.com/ExpressLRS/Backpack — GPL-3.0-only
// Copyright (c) ExpressLRS contributors
#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <cstddef>

#define RESERVED_EEPROM_SIZE 1024

class ELRS_EEPROM
{
public:
    void Begin();
    uint8_t ReadByte(const uint32_t address);
    void WriteByte(const uint32_t address, const uint8_t value);
    void Commit();

    template <typename T> void Get(uint32_t addr, T &value)
    {
        uint8_t* p = (uint8_t*)(void*)&value;
        size_t   i = sizeof(value);
        while(i--)  *p++ = ReadByte(addr++);
    };

    template <typename T> const void Put(uint32_t addr, const T &value)
    {
        const uint8_t* p = (const uint8_t*)(const void*)&value;
        size_t         i = sizeof(value);
        while(i--)  WriteByte(addr++, *p++);
    };
};
