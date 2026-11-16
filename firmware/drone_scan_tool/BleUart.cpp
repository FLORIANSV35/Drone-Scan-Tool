// This file intentionally does NOT `#define Serial IO` (unlike
// drone_scan_tool.ino) — `Serial` here is always the real hardware UART,
// so DualSerial has something concrete to mirror to.
#include "BleUart.h"
#include <NimBLEDevice.h>

// Nordic UART Service UUIDs — a de facto standard for "BLE as a serial
// port", recognised by generic BLE terminal apps too, not just our own web
// app. We only need one characteristic (NimBLEStreamServer handles both
// writes-from-client and notifies-to-client on it), unlike the usual
// separate RX/TX pair.
#define BLE_UART_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_UART_CHAR_UUID    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

static NimBLEStreamServer bleStream;

DualSerial IO;

void DualSerial::begin(unsigned long baud) {
  Serial.begin(baud);
}

int DualSerial::available() {
  int n = Serial.available();
  if (bleStream.ready()) n += bleStream.available();
  return n;
}

int DualSerial::read() {
  if (Serial.available()) return Serial.read();
  if (bleStream.ready())  return bleStream.read();
  return -1;
}

int DualSerial::peek() {
  if (Serial.available()) return Serial.peek();
  if (bleStream.ready())  return bleStream.peek();
  return -1;
}

void DualSerial::flush() {
  Serial.flush();
}

size_t DualSerial::write(uint8_t c) {
  Serial.write(c);
  if (bleStream.ready()) bleStream.write(c);
  return 1;
}

size_t DualSerial::write(const uint8_t *buffer, size_t size) {
  Serial.write(buffer, size);
  if (bleStream.ready()) bleStream.write(buffer, size);
  return size;
}

// Keeps the connection alive/responsive and restarts advertising after a
// disconnect. BLE_HS_CONN_HANDLE_NONE-style bookkeeping is handled inside
// NimBLEStreamServer itself; this is just the server-level housekeeping the
// library expects an app to do.
class BleUartServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
    // Shorter connection interval than the default: lower latency for the
    // command/response protocol (ACK:/BOOT:/etc.) than the out-of-the-box
    // params, which favour power saving over responsiveness.
    server->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 200);
  }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
    NimBLEDevice::startAdvertising();
  }
};
static BleUartServerCallbacks bleServerCallbacks;

void bleUartBegin(const char *deviceName) {
  NimBLEDevice::init(deviceName);

  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(&bleServerCallbacks);

  // 1024 bytes each way is comfortably more than our longest line (a JSON
  // drone-beacon record); NimBLEStreamServer chunks to the negotiated MTU
  // internally, so lines longer than one BLE packet aren't our problem.
  bleStream.begin(NimBLEUUID(BLE_UART_SERVICE_UUID), NimBLEUUID(BLE_UART_CHAR_UUID), 1024, 1024, false);

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_UART_SERVICE_UUID);
  adv->setName(deviceName);
  adv->enableScanResponse(true);
  adv->start();
}
