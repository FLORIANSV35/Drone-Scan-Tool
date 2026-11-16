// BLE transport for scan mode, mirroring the plain-text serial protocol
// (STEP:/RANGE:/ACK:/BOOT:/JSON drone lines/...) over a Nordic UART Service
// (NUS) -style GATT service, so the web app/APK can connect wirelessly
// where Web Serial isn't available (Android has no Web Serial — only
// WebUSB, which this board's UART bridge can't speak — so this is the only
// realistic wireless path for the Android APK).
//
// Scan-mode only, by design: backpack mode's Serial is the MSP link to
// RotorHazard and must not have anything else mixed into it.
//
// Not yet tested on hardware (no board connected while this was written).
// Needs, once it is: a real BLE central (Chrome on a computer or Android)
// connecting, checking lines arrive intact for both short (ACK:) and long
// (JSON drone/{"id":...}) lines, and checking the RX command path (typing
// RANGE:/STEP:/etc from the web app over BLE) actually reaches
// handleCommand(). See the chunking note in BleUart.cpp for the main risk
// (lines longer than the negotiated MTU).
#pragma once

#include <Arduino.h>

// DualSerial: a Stream that forwards everything written to it to the real
// hardware Serial *and* to any connected BLE central, and reads from
// whichever of the two (hardware Serial or the BLE RX characteristic) has
// bytes waiting. drone_scan_tool.ino does `#define Serial IO` right after
// including this header, so every existing Serial.print/read call in that
// file is redirected here automatically — BleUart.cpp itself does NOT
// define that macro, so it can still reach the real Serial object directly.
class DualSerial : public Stream {
public:
  void begin(unsigned long baud);

  // Stream
  int available() override;
  int read() override;
  int peek() override;
  void flush();

  // Print
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *buffer, size_t size) override;
  using Print::write;
};

extern DualSerial IO;

// Starts advertising the BLE UART service. Call once from setup(), after
// IO.begin(). Safe to call even if the BLE stack ends up not being used
// (e.g. nobody connects) — scan mode's own behaviour doesn't depend on it.
void bleUartBegin(const char *deviceName);
