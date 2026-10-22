// Adapted from the ExpressLRS Backpack project (src/Timer_main.cpp and its
// device/LED/button helpers) — see TimerBackpack.h for what was dropped.
// https://github.com/ExpressLRS/Backpack — GPL-3.0-only
// Copyright (c) ExpressLRS contributors
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <MD5Builder.h>
#include <Preferences.h>

#include "TimerBackpack.h"
#include "config.h"
#include "msp.h"
#include "msptypes.h"
#include "elrs_eeprom.h"
#include "bp_config.h"

// ═══════════════════════════════════════════════
//  State
// ═══════════════════════════════════════════════

typedef enum { bp_starting, bp_binding, bp_running } bp_state_e;

static bp_state_e connectionState = bp_starting;
static uint32_t   bindingStart    = 0;
static bool       isBinding       = false;

static ELRS_EEPROM eeprom;
static bool         storageReady = false;

static uint8_t groupAddress[6] = {0,0,0,0,0,0}; // bound ESP-NOW group address (UID)
static uint8_t sendAddress[6]  = {0,0,0,0,0,0}; // current send target (usually == groupAddress)

static MSP msp;
static esp_now_peer_info_t peerInfo;

static QueueHandle_t rxqueue = nullptr; // packets received over ESP-NOW, to forward to Serial
static QueueHandle_t txqueue = nullptr; // packets received over Serial, to forward over ESP-NOW
static SemaphoreHandle_t sendLock = nullptr;
static bool     espnowCTS    = true;
static bool     sendSuccess  = false;
static int      sendAttempt  = 0;
static const int maxAttempt  = 5;

#define BINDING_NO_PEER_TIMEOUT  120000UL  // ms with no matching peer before binding gives up

// ═══════════════════════════════════════════════
//  Storage (lazy: usable from scan mode too, for backpackSetPhrase())
// ═══════════════════════════════════════════════

static void ensureStorage() {
  if (storageReady) return;
  eeprom.Begin();
  bpConfig.SetStorageProvider(&eeprom);
  bpConfig.Load();
  memcpy(groupAddress, bpConfig.GetGroupAddress(), 6);
  memcpy(sendAddress, groupAddress, 6);
  storageReady = true;
}

void backpackSetPhrase(const char *phrase) {
  ensureStorage();

  char buf[96];
  snprintf(buf, sizeof(buf), "-DMY_BINDING_PHRASE=\"%s\"", phrase);

  MD5Builder md5;
  md5.begin();
  md5.add((uint8_t*)buf, (uint16_t)strlen(buf));
  md5.calculate();
  uint8_t digest[16];
  md5.getBytes(digest);

  memcpy(groupAddress, digest, 6);
  memcpy(sendAddress, groupAddress, 6);
  bpConfig.SetGroupAddress(groupAddress);
  bpConfig.SetPhrase(phrase); // kept only so it can be read back later (backpackGetPhrase)
  bpConfig.Commit();

  Serial.printf("ACK:PHRASE=%s,UID=%02X%02X%02X%02X%02X%02X\n",
                bpConfig.GetPhrase(),
                groupAddress[0], groupAddress[1], groupAddress[2],
                groupAddress[3], groupAddress[4], groupAddress[5]);
}

// Reads back whatever phrase was last set with backpackSetPhrase(), straight
// from the ESP's own storage — the UID alone can't be reversed into it.
const char *backpackGetPhrase() {
  ensureStorage();
  return bpConfig.GetPhrase();
}

// The web app sends this exact line right after connecting, specifically
// so this auto-detection has an unambiguous signal to match on (instead of
// only guessing from command prefixes, which — however unlikely — a binary
// MSP payload could in theory coincidentally contain). Keep this string
// identical to the one in web/index.html's connect().
#define SCAN_APP_HELLO "SCANAPP:HELLO"

// Recognises a line from the drone_scan_tool scan-mode text protocol
// (never seen from RotorHazard/ExpressLRS, which speaks binary MSP).
static bool looksLikeScanModeCommand(const char *line) {
  if (strcmp(line, SCAN_APP_HELLO) == 0) return true;
  static const char *prefixes[] = { "STEP:", "SCALE:", "RANGE:", "CAL:", "FOCUS:" };
  for (uint8_t i = 0; i < sizeof(prefixes) / sizeof(prefixes[0]); i++) {
    if (strncmp(line, prefixes[i], strlen(prefixes[i])) == 0) return true;
  }
  return strcmp(line, "DIAG") == 0;
}

// ═══════════════════════════════════════════════
//  Mode switch back to scanning (persisted, needs a reboot)
// ═══════════════════════════════════════════════

static void exitToScanMode() {
  Serial.println("ACK:MODE=SCAN");
  Preferences p;
  p.begin("rssi-scan", false);
  p.putBool("bpMode", false);
  p.end();
  delay(100);
  ESP.restart();
}

// ═══════════════════════════════════════════════
//  LED (status) — GPIO defined by PIN_BACKPACK_LED
// ═══════════════════════════════════════════════

static void ledSet(bool on) { digitalWrite(PIN_BACKPACK_LED, on ? HIGH : LOW); }

static void ledUpdate(uint32_t now) {
  static uint32_t lastToggle = 0;
  static bool     on = false;
  uint32_t period;

  switch (connectionState) {
    case bp_binding: period = 150; break;             // fast blink while binding
    default:         period = on ? 1900 : 100; break; // short heartbeat blip while running
  }
  if (now - lastToggle >= period) {
    on = !on;
    ledSet(on);
    lastToggle = now;
  }
}

static void ledBlip() {
  // Lets ledUpdate's heartbeat double as "ESP-NOW activity" by forcing an
  // immediate short on-blip; simplest is just to flash it directly here.
  ledSet(true);
}

// ═══════════════════════════════════════════════
//  Button (BOOT, GPIO0) — tap = enter binding, hold = back to scan mode
// ═══════════════════════════════════════════════

static void buttonUpdate(uint32_t now) {
  static uint32_t lastChange = 0;
  static bool     pressed    = false;
  static bool     longFired  = false;
  const uint32_t  DEBOUNCE_MS = 25;
  const uint32_t  LONG_MS     = 1500;

  bool down = (digitalRead(PIN_BACKPACK_BUTTON) == LOW); // INPUT_PULLUP, active low

  if (down != pressed) {
    if (now - lastChange > DEBOUNCE_MS) {
      pressed = down;
      lastChange = now;
      if (pressed) {
        longFired = false;
      } else if (!longFired) {
        backpackEnterBinding();
      }
    }
  } else if (pressed && !longFired && (now - lastChange > LONG_MS)) {
    longFired = true;
    exitToScanMode();
  }
}

// ═══════════════════════════════════════════════
//  ESP-NOW plumbing
// ═══════════════════════════════════════════════

static void SetSoftMACAddress() {
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  uint8_t mac[6];
  memcpy(mac, groupAddress, 6);
  mac[0] &= ~0x01; // must be a unicast address (even first byte) to be settable

  esp_wifi_set_mac(WIFI_IF_STA, mac);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
}

static void registerPeer(const uint8_t *address) {
  esp_now_del_peer(address);
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

// Adopt a group address received (over Serial or ESP-NOW) while binding.
static void adoptGroupAddress(const uint8_t *addr) {
  memcpy(groupAddress, addr, 6);
  memcpy(sendAddress, groupAddress, 6);
  bpConfig.SetGroupAddress(groupAddress);
  bpConfig.Commit();
  SetSoftMACAddress();
  registerPeer(groupAddress);
  connectionState = bp_running;
  isBinding = false;
  Serial.printf("ACK:BIND=OK,UID=%02X%02X%02X%02X%02X%02X\n",
                groupAddress[0], groupAddress[1], groupAddress[2],
                groupAddress[3], groupAddress[4], groupAddress[5]);
}

static void SendVersionResponse() {
  mspPacket_t out;
  out.reset();
  out.makeResponse();
  out.function = MSP_ELRS_GET_BACKPACK_VERSION;
  out.addByte(1); out.addByte(0); out.addByte(0); // report as backpack protocol v1.0.0
  msp.sendPacket(&out, &Serial);
}

static int sendMSPViaEspnow(mspPacket_t *packet) {
  uint8_t packetSize = msp.getTotalPacketSize(packet);
  uint8_t nowDataOutput[packetSize];
  if (!msp.convertToByteArray(packet, nowDataOutput)) return -1;
  int err = esp_now_send(sendAddress, nowDataOutput, packetSize);
  ledBlip();
  return err;
}

// A packet arrived from RotorHazard over Serial (USB), headed for the peer.
static void ProcessMSPPacketFromTimer(mspPacket_t *packet) {
  if (connectionState == bp_binding) {
    if (packet->function == MSP_ELRS_BIND) adoptGroupAddress(packet->payload);
    return;
  }

  switch (packet->function) {
    case MSP_ELRS_BACKPACK_SET_MODE:
      if (packet->payloadSize == 1 && packet->payload[0] == 'B') {
        backpackEnterBinding();
      }
      break;
    case MSP_ELRS_GET_BACKPACK_VERSION:
      SendVersionResponse();
      break;
    case MSP_ELRS_SET_SEND_UID: {
      uint8_t useCustom = packet->readByte();
      if (useCustom == 0x01 && !packet->readError) {
        for (uint8_t i = 0; i < 6; i++) sendAddress[i] = packet->readByte();
      } else {
        memcpy(sendAddress, groupAddress, 6);
      }
      registerPeer(sendAddress);
      break;
    }
    default:
      // transparently forward anything else (OSD text, race status, ...)
      sendMSPViaEspnow(packet);
      break;
  }
}

// A packet arrived from a peer over ESP-NOW, headed for RotorHazard.
static void ProcessMSPPacketFromPeer(mspPacket_t *packet) {
  if (connectionState == bp_binding) {
    if (packet->function == MSP_ELRS_BIND) adoptGroupAddress(packet->payload);
    return;
  }

  // Only the DVR-REC triggered "recording state" (race start/stop) is
  // forwarded on to RotorHazard; everything else from peers is ignored.
  if (packet->function == MSP_ELRS_BACKPACK_SET_RECORDING_STATE) {
    msp.sendPacket(packet, &Serial);
  }
}

// esp_now's callback signatures changed with arduino-esp32 core v3 (IDF 5):
// the mac address moved inside an info/tx_info struct instead of being
// passed directly. Support both so this builds on core v2 and v3.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  espnowCTS = true;
  sendSuccess = (status == ESP_NOW_SEND_SUCCESS);
  if (sendLock) xSemaphoreGive(sendLock);
}

static void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int data_len) {
  const uint8_t *mac_addr = info->src_addr;
#else
static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  espnowCTS = true;
  sendSuccess = (status == ESP_NOW_SEND_SUCCESS);
  if (sendLock) xSemaphoreGive(sendLock);
}

static void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
#endif
  static MSP recv_msp;
  bool fromKnownPeer = (connectionState == bp_binding) || (memcmp(mac_addr, groupAddress, 6) == 0);
  for (int i = 0; i < data_len; i++) {
    if (recv_msp.processReceivedByte(data[i])) {
      if (fromKnownPeer && rxqueue) {
        xQueueSend(rxqueue, recv_msp.getReceivedPacket(), (TickType_t)1024);
      }
      recv_msp.markPacketReceived();
    }
  }
  ledBlip();
}

// ═══════════════════════════════════════════════
//  Public API
// ═══════════════════════════════════════════════

void backpackEnterBinding() {
  connectionState = bp_binding;
  bindingStart = millis();
  isBinding = true;
}

void backpackSetup() {
  Serial.begin(BACKPACK_BAUD_RATE);
  delay(300);

  ensureStorage();

  pinMode(PIN_BACKPACK_LED, OUTPUT);
  pinMode(PIN_BACKPACK_BUTTON, INPUT_PULLUP);

  rxqueue  = xQueueCreate(10, sizeof(mspPacket_t));
  txqueue  = xQueueCreate(64, sizeof(mspPacket_t));
  sendLock = xSemaphoreCreateBinary();
  xSemaphoreGive(sendLock);

  SetSoftMACAddress();

  if (esp_now_init() != ESP_OK) {
    Serial.println("BOOT:MODE=BACKPACK,ESPNOW_INIT_FAILED");
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  registerPeer(groupAddress);

  connectionState = bp_running;

  Serial.printf("BOOT:MODE=BACKPACK,UID=%02X%02X%02X%02X%02X%02X\n",
                groupAddress[0], groupAddress[1], groupAddress[2],
                groupAddress[3], groupAddress[4], groupAddress[5]);
}

void backpackLoop() {
  uint32_t now = millis();

  ledUpdate(now);
  buttonUpdate(now);

  if (connectionState == bp_binding && isBinding &&
      (now - bindingStart) > BINDING_NO_PEER_TIMEOUT) {
    connectionState = bp_running;
    isBinding = false;
    Serial.println("ACK:BIND=TIMEOUT");
  }

  // Read Serial: feed the MSP parser, and in parallel watch for plain text
  // command lines (a human/our web app typing them, as opposed to the
  // binary MSP frames RotorHazard sends). The two never collide: MSP frames
  // always start with '$','X', which these lines never contain. This lets
  // you drive the mode entirely from serial, with no physical button needed.
  static char lineBuf[20];
  static uint8_t lineLen = 0;

  while (Serial.available()) {
    uint8_t c = Serial.read();

    if (msp.processReceivedByte(c)) {
      if (txqueue) xQueueSend(txqueue, msp.getReceivedPacket(), (TickType_t)1024);
      msp.markPacketReceived();
    }

    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        if (strcmp(lineBuf, "MODE:SCAN") == 0 || looksLikeScanModeCommand(lineBuf)) {
          // Either asked explicitly, or the scanner's web app just sent one
          // of its own commands (e.g. the RANGE:... it sends right after
          // connecting) — a sure sign this isn't RotorHazard on the other
          // end, so switch back without making the user ask twice.
          exitToScanMode();
        } else if (strcmp(lineBuf, "BACKPACK:BIND") == 0) {
          backpackEnterBinding();
          Serial.println("ACK:BIND=WAITING");
        }
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = (char)c;
    } else {
      lineLen = 0; // not a command we recognise, discard
    }
  }

  // Drain txqueue (Serial -> ESP-NOW), respecting ESP-NOW send back-pressure
  if (sendLock) xSemaphoreTake(sendLock, (TickType_t)512);
  if (txqueue && uxQueueMessagesWaiting(txqueue) > 0 && espnowCTS) {
    mspPacket_t packet;
    xQueuePeek(txqueue, &packet, (TickType_t)512);
    uint16_t function = packet.function;

    if (function == MSP_ELRS_GET_BACKPACK_VERSION ||
        function == MSP_ELRS_BACKPACK_SET_MODE ||
        function == MSP_ELRS_SET_SEND_UID) {
      if (sendLock) xSemaphoreGive(sendLock);
      ProcessMSPPacketFromTimer(&packet);
      xQueueReceive(txqueue, &packet, (TickType_t)512);
    } else {
      espnowCTS = false;
      if (sendLock) xSemaphoreGive(sendLock);
      if (++sendAttempt >= maxAttempt || sendSuccess) {
        sendAttempt = 0;
        ProcessMSPPacketFromTimer(&packet);
        xQueueReceive(txqueue, &packet, (TickType_t)512);
      } else {
        ProcessMSPPacketFromTimer(&packet);
      }
    }
  } else if (sendLock) {
    xSemaphoreGive(sendLock);
  }

  // Drain rxqueue (ESP-NOW -> Serial)
  if (rxqueue && uxQueueMessagesWaiting(rxqueue) > 0) {
    mspPacket_t rxPacket;
    if (xQueueReceive(rxqueue, &rxPacket, (TickType_t)512) == pdTRUE) {
      ProcessMSPPacketFromPeer(&rxPacket);
    }
  }
}
