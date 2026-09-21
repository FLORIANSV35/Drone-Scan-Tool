/*
 * RX5808 Spectrum Analyzer + Sniffer Balises Drone — V3
 * ESP32-S3 DevKit
 *
 * Scan spectre 5.8GHz (RX5808 SPI bit-bang) en parallèle avec
 * détection des balises drone DGAC France + EU Remote ID (F3411-22a).
 *
 * Sortie série mixte :
 *   5658:123          → mesure RSSI (fréquence:valeur)
 *   EOS               → fin d'un sweep complet
 *   BOOT:...          → infos au démarrage
 *   ACK:...           → acquittement commande
 *   DIAG:...          → diagnostic RSSI
 *   {"id":...}        → balise drone détectée / mise à jour (JSON)
 *   {"event":"timeout","id":"..."} → balise expirée (JSON)
 *
 * Commandes série :
 *   STEP:1/2/5        → résolution scan
 *   SCALE:min,max     → échelle Y
 *   FOCUS:XXXX        → mode focus bandes
 *   FOCUS:OFF         → scan normal
 *   DIAG              → diagnostic RSSI
 *
 * Protocoles drone :
 *   DGAC France (arrêté 27/12/2019, OUI 6A:5C:35)
 *   EU Remote ID ASTM F3411-22a (OUI FA:0B:BC, subtype 0x0D)
 *   EU Remote ID ASTM F3411-19 legacy (OUI 5F:04:01)
 */

#include "config.h"
#include <Preferences.h>
#include <WiFi.h>
#include "esp_wifi.h"

// ═══════════════════════════════════════════════
//  SCANNER — variables globales
// ═══════════════════════════════════════════════

Preferences prefs;

uint16_t savedScaleMin = 0;
uint16_t savedScaleMax = 0;
bool     scaleValid    = false;
int16_t  calOffsets[8] = {0,0,0,0,0,0,0,0};
bool     calValid      = false;

uint16_t freqTable[MAX_FREQ];
uint16_t numFreq  = 0;
uint8_t  freqStep = DEFAULT_FREQ_STEP;
bool     focusMode = false;
uint16_t rangeMin = FMIN;   // plage de sweep (modifiable via RANGE:)
uint16_t rangeMax = FMAX;

char    cmdBuf[32];
uint8_t cmdLen = 0;

const uint16_t CHAN_FREQS[8][8] = {
  {5658,5695,5732,5769,5806,5843,5880,5917}, // Raceband
  {5733,5752,5771,5790,5809,5828,5847,5866}, // Band B
  {5705,5685,5665,5645,5885,5905,5925,5945}, // Band E
  {5740,5760,5780,5800,5820,5840,5860,5880}, // Fatshark F
  {5865,5845,5825,5805,5785,5765,5745,5725}, // Band A
  {5768,5804,5839,   0,   0,   0,   0,   0}, // DJI O3 CE
  {5735,5770,5805,5839,   0,   0,   0,   0}, // DJI Vista CE
  {5658,5695,5732,5769,5806,5843,5880,5917}, // DJI O4 Race
};

// ═══════════════════════════════════════════════
//  SNIFFER DRONE — OUI / types
// ═══════════════════════════════════════════════

static const uint8_t DGAC_OUI[3]    = {0x6A, 0x5C, 0x35};
static const uint8_t DGAC_VS_TYPE   = 0x01;
static const uint8_t ASTM_OUI_V2[3] = {0xFA, 0x0B, 0xBC};
static const uint8_t ASTM_SUBTYPE   = 0x0D;
static const uint8_t ASTM_OUI_V1[3] = {0x5F, 0x04, 0x01};

static const char* UA_TYPE_STR[] = {
  "Inconnu","Avion","Hélicoptère/Multirotor","Gyroplane",
  "Hybride VTOL","Ornithoptère","Planeur","Cerf-volant",
  "Ballon libre","Ballon captif","Dirigeable","Parachute",
  "Fusée","Aéronef tracté","Obstacle sol","Autre"
};
static const char* EU_CATEGORY_STR[] = {"Inconnu","Open","Specific","Certified"};

// ═══════════════════════════════════════════════
//  SNIFFER — queue ISR → loop
// ═══════════════════════════════════════════════

struct PktEntry {
  uint8_t data[PKT_BUF_SIZE];
  uint8_t mac[6];
  uint8_t len;
  int8_t  rssi;
  uint8_t proto; // 0=DGAC, 1=EU-RID
};

static PktEntry  pktQueue[PKT_QUEUE_LEN];
static volatile uint8_t pktHead = 0;
static volatile uint8_t pktTail = 0;

inline bool queueFull()  { return ((pktHead + 1) % PKT_QUEUE_LEN) == pktTail; }
inline bool queueEmpty() { return pktHead == pktTail; }

// ═══════════════════════════════════════════════
//  SNIFFER — structure balise
// ═══════════════════════════════════════════════

#define DUMP_SIZE (PKT_BUF_SIZE * 3 + 1)

struct BaliseDGAC {
  char     id[32];
  char     id_const[32];
  char     state[16];
  char     dump[DUMP_SIZE];
  double   lat, lon;
  double   home_lat, home_lon;
  double   dist_home;
  int16_t  alt, haut;
  bool     has_alt, has_haut;
  uint16_t cap;
  uint8_t  vit;
  int8_t   speed_v;
  uint32_t lastSeen;
  int8_t   rssi;
  char     mac_str[18];
  uint8_t  eu_ua_type, eu_id_type, eu_category, eu_class, eu_op_status;
  double   op_lat, op_lon;
  int16_t  op_alt;
  char     eu_op_id[21];
  char     eu_self_desc[24];
  uint32_t eu_timestamp;
  bool     eu_loc_valid, eu_sys_valid;
};

// Prototypes
void parse_eurid_msg(BaliseDGAC &b, const uint8_t *msg, uint8_t msg_len);
void process_eurid(const uint8_t *p, uint8_t len, const uint8_t *mac, int rssi);
void process_dgac(const uint8_t *tlv, uint8_t len, int rssi);

BaliseDGAC balises[MAX_BALISES];
uint8_t    baliseCount = 0;
uint32_t   lastClean   = 0;


// ═══════════════════════════════════════════════
//  SNIFFER — utils
// ═══════════════════════════════════════════════

inline double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dlat = radians(lat2 - lat1);
  double dlon = radians(lon2 - lon1) * cos(radians((lat1+lat2)*0.5));
  return R * sqrt(dlat*dlat + dlon*dlon);
}

inline int32_t  i32be(const uint8_t *p) { return (int32_t)((p[0]<<24)|(p[1]<<16)|(p[2]<<8)|p[3]); }
inline int16_t  i16be(const uint8_t *p) { return (int16_t)((p[0]<<8)|p[1]); }
inline uint16_t u16be(const uint8_t *p) { return (uint16_t)((p[0]<<8)|p[1]); }

inline int32_t i32le(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24));
}
inline uint32_t u32le(const uint8_t *p) {
  return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}
inline uint16_t u16le(const uint8_t *p) { return (uint16_t)(p[0]|(p[1]<<8)); }

inline float eu_alt(const uint8_t *p) {
  uint16_t raw = u16le(p);
  if (raw == 0xFFFF) return -9999.0f;
  return raw * 0.5f - 1000.0f;
}
inline double eu_coord(const uint8_t *p) { return i32le(p) / 1e7; }

inline void safecopy(char *dst, const uint8_t *src, int len, int maxlen) {
  int n = (len < maxlen-1) ? len : maxlen-1;
  memcpy(dst, src, n); dst[n] = '\0';
}

// ═══════════════════════════════════════════════
//  SNIFFER — gestion balises
// ═══════════════════════════════════════════════

int findBalise(const char *id) {
  for (int i = 0; i < baliseCount; i++)
    if (strcmp(balises[i].id, id) == 0) return i;
  return -1;
}
int findBaliseByMac(const char *mac) {
  for (int i = 0; i < baliseCount; i++)
    if (strcmp(balises[i].mac_str, mac) == 0) return i;
  return -1;
}
int allocBalise(const char *id) {
  int idx = findBalise(id);
  if (idx >= 0) return idx;
  // Nouvelle balise — bip buzzer
  if (baliseCount < MAX_BALISES) { idx = baliseCount++; }
  else {
    idx = 0;
    for (int i = 1; i < MAX_BALISES; i++)
      if (balises[i].lastSeen < balises[idx].lastSeen) idx = i;
  }
  memset(&balises[idx], 0, sizeof(BaliseDGAC));
  return idx;
}
void removeBalise(int idx) {
  Serial.printf("{\"event\":\"timeout\",\"id\":\"%s\"}\n", balises[idx].id);
  for (int i = idx; i < baliseCount-1; i++) balises[i] = balises[i+1];
  baliseCount--;
}

// ═══════════════════════════════════════════════
//  SNIFFER — parser DGAC TLV
// ═══════════════════════════════════════════════

void process_dgac(const uint8_t *tlv, uint8_t len, int rssi) {
  char    id[32]={}, idc[32]={};
  double  lat=0, lon=0, hl=0, ho=0;
  int16_t alt=0, haut=0;
  uint8_t vit=0; uint16_t cap=0;
  bool v01=0,v02=0,v03=0,v04=0,v05=0,v06=0,v07=0,v08=0,v09=0,v0A=0,v0B=0;

  for (int p=0; p+2<=len;) {
    uint8_t t=tlv[p++], l=tlv[p++];
    if (p+l>len) break;
    const uint8_t *v=&tlv[p];
    switch(t) {
      case 0x01: if(l&&v[0]==1) v01=1; break;
      case 0x02: safecopy(id,  v,l,32); v02=1; break;
      case 0x03: safecopy(idc, v,l,32); v03=1; break;
      case 0x04: lat  = i32be(v)/100000.0; v04=1; break;
      case 0x05: lon  = i32be(v)/100000.0; v05=1; break;
      case 0x06: alt  = i16be(v); v06=1; break;
      case 0x07: haut = i16be(v); v07=1; break;
      case 0x08: hl   = i32be(v)/100000.0; v08=1; break;
      case 0x09: ho   = i32be(v)/100000.0; v09=1; break;
      case 0x0A: vit  = v[0]; v0A=1; break;
      case 0x0B: cap  = u16be(v); v0B=1; break;
    }
    p+=l;
  }

  bool has_id  = v02 || v03;
  bool has_alt = v06 || v07;
  const char *state;
  if (v01&&has_id&&v04&&v05&&has_alt&&v08&&v09&&v0A&&v0B) state="DGAC";
  else if (has_id&&v04&&v05) state="DGAC-WARN";
  else state="ERROR";

  const char *used_id = v02 ? id : (v03 ? idc : nullptr);
  if (!used_id || !used_id[0]) return;

  int idx = allocBalise(used_id);
  BaliseDGAC &b = balises[idx];
  strncpy(b.id,       used_id, 31); b.id[31]=0;
  strncpy(b.id_const, v03?idc:"", 31); b.id_const[31]=0;
  strncpy(b.state,    state,   15); b.state[15]=0;
  b.lat=lat; b.lon=lon; b.alt=alt; b.haut=haut;
  b.has_alt=v06; b.has_haut=v07;
  b.vit=vit; b.cap=cap;
  b.home_lat=hl; b.home_lon=ho;
  b.dist_home = distanceMeters(hl,ho,lat,lon);
  b.rssi=(int8_t)rssi; b.lastSeen=millis();

  int dp=0;
  for (int i=0;i<len&&dp<(int)sizeof(b.dump)-4;i++)
    dp+=sprintf(b.dump+dp,"%02X ",tlv[i]);
  if(dp>0) b.dump[dp-1]='\0';

  Serial.printf(
    "{\"id\":\"%s\",\"lat\":%.5f,\"lon\":%.5f,"
    "\"alt\":%d,\"alt_type\":\"%s\","
    "\"speed\":%d,\"heading\":%d,"
    "\"home_lat\":%.5f,\"home_lon\":%.5f,"
    "\"rssi\":%d,\"state\":\"%s\","
    "\"dump\":\"%s\",\"id_const\":\"%s\"}\n",
    b.id, b.lat, b.lon,
    b.has_haut?b.haut:b.alt, b.has_haut?"hauteur":"altitude",
    b.vit, b.cap, b.home_lat, b.home_lon,
    b.rssi, b.state, b.dump, b.id_const
  );
}

// ═══════════════════════════════════════════════
//  SNIFFER — parser EU Remote ID F3411-22a
// ═══════════════════════════════════════════════

void parse_eurid_msg(BaliseDGAC &b, const uint8_t *msg, uint8_t msg_len) {
  if (msg_len < 25) return;
  uint8_t msg_type = (msg[0] >> 4) & 0x0F;

  switch (msg_type) {
    case 0x0: { // Basic ID
      b.eu_id_type = (msg[1] >> 4) & 0x0F;
      b.eu_ua_type = msg[1] & 0x0F;
      char new_id[21] = {};
      memcpy(new_id, &msg[2], 20); new_id[20]=0;
      for (int i=19; i>=0 && new_id[i]=='\0'; i--) new_id[i]=0;
      if (new_id[0]) { strncpy(b.id, new_id, 31); b.id[31]=0; }
      break;
    }
    case 0x1: { // Location / Vector
      uint8_t flags   = msg[1];
      b.eu_op_status  = (flags >> 4) & 0x0F;
      uint8_t ew      = (flags >> 2) & 0x01;
      uint8_t sp_mult = (flags >> 1) & 0x01;
      uint8_t dir_raw = msg[2];
      if (dir_raw <= 180) b.cap = (uint16_t)(dir_raw + (ew ? 180 : 0));
      uint8_t spd_raw = msg[3];
      if (spd_raw != 255) {
        float spd = sp_mult ? ((spd_raw + 255) * 0.75f) : (spd_raw * 0.25f);
        b.vit = (uint8_t)min(spd, 255.0f);
      }
      b.speed_v = (int8_t)msg[4];
      double lat = eu_coord(&msg[5]);
      double lon = eu_coord(&msg[9]);
      if (lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180) { b.lat=lat; b.lon=lon; }
      float alt_g = eu_alt(&msg[15]);
      float alt_p = eu_alt(&msg[13]);
      float height = eu_alt(&msg[17]);
      if (alt_g > -9999.0f)       { b.alt  = (int16_t)alt_g; b.has_alt  = true; }
      else if (alt_p > -9999.0f)  { b.alt  = (int16_t)alt_p; b.has_alt  = true; }
      if (height > -9999.0f)      { b.haut = (int16_t)height; b.has_haut = true; }
      b.eu_loc_valid = true;
      break;
    }
    case 0x3: { // Self-ID
      memcpy(b.eu_self_desc, &msg[2], 23); b.eu_self_desc[23]=0;
      if (b.eu_self_desc[0] && !b.id_const[0])
        strncpy(b.id_const, b.eu_self_desc, 31);
      break;
    }
    case 0x4: { // System
      double op_lat = eu_coord(&msg[2]);
      double op_lon = eu_coord(&msg[6]);
      if (op_lat >= -90 && op_lat <= 90 && op_lon >= -180 && op_lon <= 180) {
        b.op_lat = op_lat; b.op_lon = op_lon;
        if (b.home_lat == 0 && b.home_lon == 0) { b.home_lat = op_lat; b.home_lon = op_lon; }
      }
      b.eu_category = (msg[17] >> 4) & 0x0F;
      b.eu_class    = msg[17] & 0x0F;
      float op_a = eu_alt(&msg[18]);
      if (op_a > -9999.0f) b.op_alt = (int16_t)op_a;
      b.eu_timestamp = u32le(&msg[20]);
      b.eu_sys_valid = true;
      break;
    }
    case 0x5: { // Operator ID
      memcpy(b.eu_op_id, &msg[2], 20); b.eu_op_id[20]=0;
      strncpy(b.id_const, b.eu_op_id, 31); b.id_const[31]=0;
      break;
    }
    default: break;
  }
}

void process_eurid(const uint8_t *p, uint8_t len, const uint8_t *mac, int rssi) {
  if (len < 1) return;
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  int idx = findBaliseByMac(mac_str);
  if (idx < 0) {
    idx = allocBalise(mac_str);
    strncpy(balises[idx].id,      mac_str, 31); balises[idx].id[31]=0;
    strncpy(balises[idx].state,   "EU-DETECT", 15);
    strncpy(balises[idx].mac_str, mac_str, 17); balises[idx].mac_str[17]=0;
  }

  BaliseDGAC &b = balises[idx];
  b.rssi = (int8_t)rssi; b.lastSeen = millis();

  uint8_t msg_type = (p[0] >> 4) & 0x0F;
  if (msg_type == 0xF) {
    if (len < 3) return;
    uint8_t msg_size = p[1], msg_count = p[2];
    if (msg_size != 25) return;
    for (int i = 0; i < msg_count; i++) {
      uint8_t offset = 3 + i * 25;
      if (offset + 25 > len) break;
      parse_eurid_msg(b, p + offset, 25);
    }
  } else {
    parse_eurid_msg(b, p, len);
  }

  bool has_pos = (b.lat != 0 || b.lon != 0);
  bool has_id  = (b.id[0] && strcmp(b.id, mac_str) != 0);
  if      (has_id && has_pos) strncpy(b.state, "EU-RID",    15);
  else if (has_id || has_pos) strncpy(b.state, "EU-PART",   15);
  else                        strncpy(b.state, "EU-DETECT", 15);
  b.state[15]=0;

  if (has_pos && (b.home_lat != 0 || b.home_lon != 0))
    b.dist_home = distanceMeters(b.home_lat, b.home_lon, b.lat, b.lon);

  int dp=0;
  for (int i=0;i<len&&dp<(int)sizeof(b.dump)-4;i++)
    dp+=sprintf(b.dump+dp,"%02X ",p[i]);
  if(dp>0) b.dump[dp-1]='\0';

  const char *ua_str  = (b.eu_ua_type < 16) ? UA_TYPE_STR[b.eu_ua_type]  : "Inconnu";
  const char *cat_str = (b.eu_category < 4)  ? EU_CATEGORY_STR[b.eu_category] : "Inconnu";

  Serial.printf(
    "{\"id\":\"%s\",\"lat\":%.5f,\"lon\":%.5f,"
    "\"alt\":%d,\"alt_type\":\"%s\","
    "\"speed\":%d,\"heading\":%d,\"speed_v\":%.1f,"
    "\"home_lat\":%.5f,\"home_lon\":%.5f,"
    "\"op_lat\":%.5f,\"op_lon\":%.5f,"
    "\"rssi\":%d,\"state\":\"%s\","
    "\"eu_ua_type\":%d,\"eu_ua_str\":\"%s\","
    "\"eu_category\":%d,\"eu_cat_str\":\"%s\","
    "\"eu_class\":%d,\"eu_op_id\":\"%s\","
    "\"eu_timestamp\":%u,\"mac\":\"%s\","
    "\"dump\":\"%s\",\"id_const\":\"%s\"}\n",
    b.id, b.lat, b.lon,
    b.has_haut?b.haut:b.alt, b.has_haut?"hauteur":"altitude",
    b.vit, b.cap, b.speed_v*0.5f,
    b.home_lat, b.home_lon,
    b.op_lat, b.op_lon,
    b.rssi, b.state,
    b.eu_ua_type, ua_str, b.eu_category, cat_str,
    b.eu_class, b.eu_op_id,
    b.eu_timestamp, b.mac_str,
    b.dump, b.id_const
  );
}

// ═══════════════════════════════════════════════
//  WIFI CALLBACK — IRAM (ultra-léger, pas de Serial ici)
// ═══════════════════════════════════════════════

extern "C" void IRAM_ATTR wifi_sniffer_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t *f = pkt->payload;
  if ((f[0] & 0xF0) != 0x80) return; // beacon uniquement
  int rssi = pkt->rx_ctrl.rssi;
  const uint8_t *src_mac = f + 10;
  const uint8_t *ies = f + 36;
  const uint8_t *end = f + pkt->rx_ctrl.sig_len;

  while (ies + 2 <= end) {
    uint8_t ie_id = ies[0], ie_len = ies[1];
    const uint8_t *v = ies + 2;
    if (v + ie_len > end) break;

    if (ie_id == 0xDD && ie_len >= 4) {
      uint8_t proto = 0xFF;
      const uint8_t *payload = nullptr;
      uint8_t plen = 0;

      if (ie_len >= 5 && !memcmp(v, DGAC_OUI, 3) && v[3] == DGAC_VS_TYPE)
        { proto=0; payload=v+4; plen=ie_len-4; }
      else if (ie_len >= 5 && !memcmp(v, ASTM_OUI_V2, 3) && v[3] == ASTM_SUBTYPE)
        { proto=1; payload=v+4; plen=ie_len-4; }
      else if (!memcmp(v, ASTM_OUI_V1, 3))
        { proto=1; payload=v+3; plen=ie_len-3; }

      if (proto != 0xFF && plen > 0 && plen <= PKT_BUF_SIZE && !queueFull()) {
        uint8_t slot = pktHead;
        pktQueue[slot].len   = plen;
        pktQueue[slot].rssi  = (int8_t)rssi;
        pktQueue[slot].proto = proto;
        memcpy(pktQueue[slot].data, payload, plen);
        if (proto == 1) memcpy(pktQueue[slot].mac, src_mac, 6);
        pktHead = (pktHead+1) % PKT_QUEUE_LEN;
      }
    }
    ies = v + ie_len;
  }
}

// ═══════════════════════════════════════════════
//  Traitement queue + nettoyage (appelé dans loop)
// ═══════════════════════════════════════════════

void processQueue() {
  while (!queueEmpty()) {
    uint8_t slot = pktTail;
    pktTail = (pktTail+1) % PKT_QUEUE_LEN;
    PktEntry &e = pktQueue[slot];
    if (e.proto == 0) process_dgac(e.data, e.len, e.rssi);
    else              process_eurid(e.data, e.len, e.mac, e.rssi);
  }
}

void cleanBalises() {
  if (millis() - lastClean < CLEAN_INTERVAL) return;
  lastClean = millis();
  for (int i = 0; i < baliseCount; i++) {
    if (millis() - balises[i].lastSeen > BEACON_TIMEOUT) {
      removeBalise(i); i--;
    }
  }
}

// ═══════════════════════════════════════════════
//  SCANNER — SPI RX5808 bit-bang
// ═══════════════════════════════════════════════

#define TSPI 100

void spiSendBit(bool high) {
  digitalWrite(PIN_DATA, high ? HIGH : LOW);
  delayMicroseconds(TSPI);
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(TSPI);
  digitalWrite(PIN_CLK, LOW);
  delayMicroseconds(TSPI);
}
void spiEnableHigh() { digitalWrite(PIN_LE, HIGH); delayMicroseconds(TSPI); }
void spiEnableLow()  { digitalWrite(PIN_LE, LOW);  delayMicroseconds(TSPI); }

uint16_t freqToReg(uint16_t freqMHz) {
  uint16_t tf = (freqMHz - 479) / 2;
  return ((tf/32) << 7) | (tf%32);
}

void setFrequency(uint16_t freqMHz) {
  uint16_t vtxHex = freqToReg(freqMHz);
  spiEnableHigh(); spiEnableLow();
  spiSendBit(1); spiSendBit(0); spiSendBit(0); spiSendBit(0); spiSendBit(1);
  for (uint8_t i=0; i<16; i++) { spiSendBit(vtxHex & 0x1); vtxHex >>= 1; }
  for (uint8_t i=0; i<4; i++) spiSendBit(0);
  spiEnableHigh(); delay(2);
  digitalWrite(PIN_CLK, LOW); digitalWrite(PIN_DATA, LOW);
}

void resetRxModule() {
  spiEnableHigh(); spiEnableLow();
  spiSendBit(1); spiSendBit(1); spiSendBit(1); spiSendBit(1); spiSendBit(1);
  for (uint8_t i=0; i<20; i++) spiSendBit(0);
  spiEnableHigh(); delay(10);
  // power setup
  uint32_t options = 0b11010000110111110011UL;
  spiEnableHigh(); spiEnableLow();
  spiSendBit(0); spiSendBit(1); spiSendBit(0); spiSendBit(1); spiSendBit(1);
  for (uint8_t i=0; i<20; i++) { spiSendBit(options & 0x1); options >>= 1; }
  spiEnableHigh();
}

// ═══════════════════════════════════════════════
//  SCANNER — commandes série
// ═══════════════════════════════════════════════

uint8_t hexNibble(char c) {
  if (c>='0'&&c<='9') return c-'0';
  if (c>='A'&&c<='F') return c-'A'+10;
  if (c>='a'&&c<='f') return c-'a'+10;
  return 0;
}

void rebuildTable() {
  numFreq = (rangeMax - rangeMin) / freqStep + 1;
  for (uint16_t i=0; i<numFreq; i++) freqTable[i] = rangeMin + (uint16_t)i * freqStep;
}

void checkCommand() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c=='\n'||c=='\r') {
      if (cmdLen > 0) { cmdBuf[cmdLen]='\0'; handleCommand(cmdBuf); cmdLen=0; }
    } else if (cmdLen < (sizeof(cmdBuf)-1)) { cmdBuf[cmdLen++]=c; }
  }
}

void handleCommand(const char* cmd) {
  if (strncmp(cmd,"STEP:",5)==0) {
    uint8_t s=(uint8_t)atoi(cmd+5);
    if (s==1||s==2||s==5) { freqStep=s; rebuildTable(); prefsSave(); }
    Serial.print("ACK:STEP="); Serial.println(freqStep); return;
  }
  if (strncmp(cmd,"SCALE:",6)==0) {
    const char *p=cmd+6; uint16_t mn=(uint16_t)atoi(p);
    const char *comma=strchr(p,',');
    if (comma) { uint16_t mx=(uint16_t)atoi(comma+1);
      if (mx>mn) { savedScaleMin=mn; savedScaleMax=mx; scaleValid=true; prefsSave();
        Serial.printf("ACK:SCALE=%d,%d\n",mn,mx); } }
    return;
  }
  if (strncmp(cmd,"RANGE:",6)==0) {
    const char *p=cmd+6; uint16_t mn=(uint16_t)atoi(p);
    const char *comma=strchr(p,',');
    if (comma) { uint16_t mx=(uint16_t)atoi(comma+1);
      if (mx>mn && mn>=FMIN && mx<=FMAX) {
        bool changed = (mn!=rangeMin || mx!=rangeMax);
        rangeMin=mn; rangeMax=mx;
        if (changed && !focusMode) rebuildTable();
        if (changed) prefsSave();
        Serial.printf("ACK:RANGE=%d,%d\n",rangeMin,rangeMax); } }
    return;
  }
  if (strncmp(cmd,"CAL:",4)==0) {
    const char *p=cmd+4;
    for (uint8_t i=0;i<8;i++) { calOffsets[i]=(int16_t)atoi(p); p=strchr(p,','); if(!p) break; p++; }
    calValid=true; prefsSave(); Serial.println("ACK:CAL=SAVED"); return;
  }
  if (strncmp(cmd,"FOCUS:",6)==0) {
    const char *p=cmd+6;
    if (strcmp(p,"OFF")==0) { focusMode=false; rebuildTable(); Serial.println("ACK:FOCUS=OFF"); return; }
    numFreq=0;
    for (uint8_t b=0;b<8;b++) {
      uint8_t mask=(hexNibble(p[b*2])<<4)|hexNibble(p[b*2+1]);
      for (uint8_t ch=0;ch<8;ch++) {
        if (mask&(1<<ch)) { uint16_t freq=CHAN_FREQS[b][ch]; if(freq>=FMIN) freqTable[numFreq++]=freq; }
      }
    }
    for (uint8_t i=1;i<numFreq;i++) {
      uint16_t key=freqTable[i]; int8_t j=i-1;
      while(j>=0&&freqTable[j]>key){freqTable[j+1]=freqTable[j];j--;} freqTable[j+1]=key;
    }
    focusMode=(numFreq>0); Serial.printf("ACK:FOCUS=%d\n",numFreq); return;
  }
if (strcmp(cmd,"DIAG")==0) {
    const uint16_t tf[]={5658,5695,5732,5760,5800,5820,5843,5880,5905,5945};
    Serial.print("DIAG:");
    for (uint8_t i=0;i<10;i++) {
      setFrequency(tf[i]); delay(40);
      uint32_t sum=0;
      for (uint8_t s=0;s<10;s++){sum+=analogRead(PIN_RSSI);delay(2);}
      Serial.printf("%d=%d",tf[i],(int)(sum/10));
      if(i<9) Serial.print(',');
    }
    Serial.println(); return;
  }
}

// ═══════════════════════════════════════════════
//  PREFERENCES
// ═══════════════════════════════════════════════

void prefsLoad() {
  prefs.begin("rssi-scan", true);
  freqStep = prefs.getUChar("step", DEFAULT_FREQ_STEP);
  if (freqStep!=1&&freqStep!=2&&freqStep!=5) freqStep=DEFAULT_FREQ_STEP;
  uint16_t rmin = prefs.getUShort("rmin", FMIN);
  uint16_t rmax = prefs.getUShort("rmax", FMAX);
  if (rmin>=FMIN && rmax<=FMAX && rmax>rmin) { rangeMin=rmin; rangeMax=rmax; }
  savedScaleMin = prefs.getUShort("smin", 0);
  savedScaleMax = prefs.getUShort("smax", 0);
  scaleValid = (savedScaleMax > savedScaleMin);
if (prefs.getBool("calvalid", false)) {
    char key[6];
    for (uint8_t i=0;i<8;i++) { snprintf(key,sizeof(key),"cal%d",i); calOffsets[i]=prefs.getShort(key,0); }
    calValid=true;
  }
  prefs.end();
}

void prefsSave() {
  prefs.begin("rssi-scan", false);
  prefs.putUChar("step", freqStep);
  prefs.putUShort("rmin", rangeMin);
  prefs.putUShort("rmax", rangeMax);
  prefs.putUShort("smin", savedScaleMin);
  prefs.putUShort("smax", savedScaleMax);
if (calValid) {
    prefs.putBool("calvalid", true);
    char key[6];
    for (uint8_t i=0;i<8;i++) { snprintf(key,sizeof(key),"cal%d",i); prefs.putShort(key,calOffsets[i]); }
  }
  prefs.end();
}

// ═══════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════

void setup() {
  pinMode(PIN_CLK,    OUTPUT);
  pinMode(PIN_DATA,   OUTPUT);
  pinMode(PIN_LE,     OUTPUT);
  pinMode(PIN_RSSI,   INPUT);
analogReadResolution(ADC_RESOLUTION);

  Serial.begin(BAUD_RATE);
  delay(500); // attendre USB CDC

  prefsLoad();
  rebuildTable();

  // Init sniffer WiFi (mode station + promiscuous)
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect(true, true);
  delay(200);
  esp_wifi_start();
  wifi_promiscuous_filter_t flt;
  flt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
  esp_wifi_set_promiscuous_filter(&flt);
  esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_cb);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_SNIFFER_CHANNEL, WIFI_SECOND_CHAN_NONE);

  resetRxModule();
  setFrequency(5800);
  delay(50);

  uint32_t sum = 0;
  for (uint8_t i=0;i<20;i++) { sum+=analogRead(PIN_RSSI); delay(5); }
  uint16_t baseline = sum/20;

  Serial.printf("BOOT:RSSI_BASELINE=%d,STEP=%d,CHANNELS=%d",baseline,freqStep,numFreq);
  if (rangeMin!=FMIN || rangeMax!=FMAX) Serial.printf(",RANGE=%d,%d",rangeMin,rangeMax);
  if (scaleValid) Serial.printf(",SCALE=%d,%d",savedScaleMin,savedScaleMax);
  if (calValid) {
    Serial.print(",CAL=");
    for (uint8_t i=0;i<8;i++) { Serial.print(calOffsets[i]); if(i<7) Serial.print(','); }
  }
  Serial.println();
}

// ═══════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════

void loop() {
  checkCommand();

  for (uint16_t i = 0; i < numFreq; i++) {
    setFrequency(freqTable[i]);

    // Traiter les paquets drone pendant le temps de stabilisation PLL
    uint32_t t0 = millis();
    while (millis() - t0 < PLL_SETTLE_MS) {
      processQueue();
      delayMicroseconds(500);
    }

    uint32_t sum = 0;
    for (uint8_t s = 0; s < RSSI_SAMPLES; s++) {
      sum += analogRead(PIN_RSSI);
      delayMicroseconds(200);
    }
    Serial.printf("%d:%d\n", freqTable[i], (int)(sum / RSSI_SAMPLES));
  }
  Serial.println("EOS");

  // Nettoyage balises expirées + traitement queue résiduelle
  processQueue();
  cleanBalises();
}
