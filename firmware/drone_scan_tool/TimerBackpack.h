// RotorHazard Timer Backpack mode.
//
// Adapted from the ExpressLRS Backpack project (src/Timer_main.cpp and the
// small device/LED/button helpers it uses), trimmed down to what a DIY
// ESP32-S3 timer backpack actually needs: a serial (USB) <-> ESP-NOW MSP
// bridge between RotorHazard and the pilots' VRx backpacks.
// https://github.com/ExpressLRS/Backpack — GPL-3.0-only
// Copyright (c) ExpressLRS contributors
//
// Dropped on purpose (not needed for this role, and not reimplemented):
//   - the ESP8266 code paths (this sketch only targets ESP32-S3)
//   - the Wi-Fi OTA update / captive portal (reflash over USB instead, same
//     as the drone_scan_tool firmware)
//   - the compile-time "binding phrase" baked into the binary by the
//     ExpressLRS Configurator/python build scripts (no such build step
//     here) — replaced by backpackSetPhrase(), computed at runtime with
//     the exact same MD5 scheme, so pairing with an officially-built
//     backpack using the same phrase still works.
#pragma once

// Called once when switching into backpack mode (after MODE:BACKPACK +
// reboot). Takes over Serial entirely (460800 baud, MSP framing) until the
// device is rebooted back into scan mode.
void backpackSetup();

// Called every loop() iteration while in backpack mode.
void backpackLoop();

// Derive this device's ESP-NOW group address (UID) from a human-readable
// binding phrase and persist it, using the same
// MD5('-DMY_BINDING_PHRASE="<phrase>"')[0:6] scheme as the official
// ExpressLRS Configurator. Can be called from scan mode, before switching.
void backpackSetPhrase(const char *phrase);

// Reads back the phrase last set with backpackSetPhrase(), from the ESP's
// own storage (empty string if none was ever set). The group address alone
// can't be reversed back into it, which is why it's kept separately.
const char *backpackGetPhrase();

// Puts the backpack into ESP-NOW binding mode: it will adopt the group
// address sent by the next peer it hears from while binding. Also reachable
// with a short press of the BOOT button (GPIO0) while in backpack mode.
void backpackEnterBinding();

// Which mode to boot into, persisted to flash (Preferences/NVS). Survives
// everything, including a real power cycle — necessary on this board,
// since merely opening the serial port (what RotorHazard does) or
// changing its baud rate resets the chip's EN pin, which would clear any
// RTC-memory-only flag. Switch back explicitly (MODE:SCAN / the web app's
// button) rather than relying on a power cycle to reset it.
void backpackSetPersistedMode(bool backpackMode);
bool backpackGetPersistedMode();
