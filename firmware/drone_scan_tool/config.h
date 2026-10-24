#ifndef CONFIG_H
#define CONFIG_H

// ═══════════════════════════════════════════════
//  CONFIGURATION — RX5808 Spectrum Analyzer V3
//  Cible : ESP32-S3 DevKit
//  Fonctions : Scan spectre 5.8GHz + Sniffer DGAC/EU-RID
// ═══════════════════════════════════════════════

// ── Broches RX5808 ─────────────────────────────
//  CH1 (DATA) → GPIO 4
//  CH2 (LE)   → GPIO 5
//  CH3 (CLK)  → GPIO 6
//  RSSI       → GPIO 1  (ADC1_CH0)
//  VCC        → 3.3V
//  GND        → GND
//
//  GPIO 19/20 réservés USB CDC (D-/D+)
//  ADC2 incompatible WiFi → toujours utiliser ADC1 (GPIO 1-10)
#define PIN_DATA   4
#define PIN_LE     5
#define PIN_CLK    6
#define PIN_RSSI   1    // ADC1_CH0

// ── Broches mode RotorHazard Timer Backpack ────
//  (inutilisées en mode scan, ignorées si le câblage RX5808 reste branché)
//  LED status  → GPIO 21
//  Bouton bind → GPIO 0  (bouton BOOT de la carte)
#define PIN_BACKPACK_LED     21
#define PIN_BACKPACK_BUTTON  0
#define BACKPACK_BAUD_RATE   460800   // débit imposé par le protocole ExpressLRS Backpack

// Bind phrase utilisée si aucune n'a encore été définie via
// BACKPACK:PHRASE: (ou après un effacement complet de la flash).
// Mets ta vraie phrase via BACKPACK:PHRASE: (ou le panneau web) plutôt
// qu'ici : elle reste alors uniquement sur l'ESP (EEPROM), jamais dans ce
// fichier ni sur GitHub. Ne mets pas ta vraie bind phrase dans ce define,
// ce fichier est public.
#define DEFAULT_BIND_PHRASE  "CHANGE-ME"

// ── Plage de scan spectre ──────────────────────
#define FMIN       5645
#define FMAX       5945
#define DEFAULT_FREQ_STEP  5
#define MAX_FREQ  ((FMAX - FMIN) + 1)

// ── Qualité de mesure ──────────────────────────
#define RSSI_SAMPLES   10
#define PLL_SETTLE_MS  42

// ── ADC (10 bits pour compatibilité web app) ───
#define ADC_RESOLUTION  10

// ── Série ──────────────────────────────────────
#define BAUD_RATE  115200

// ── Sniffer balises drone ──────────────────────
#define WIFI_SNIFFER_CHANNEL   6     // Canal 2.4GHz (6 = 2.437 GHz, courant FR/EU)
#define MAX_BALISES            8
#define BEACON_TIMEOUT         30000  // ms sans paquet → suppression
#define CLEAN_INTERVAL          5000  // ms entre nettoyages
#define PKT_BUF_SIZE           128
#define PKT_QUEUE_LEN           16

#endif
