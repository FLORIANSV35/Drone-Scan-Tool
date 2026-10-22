# Drone Scan Tool

A 5.8 GHz (FPV) spectrum analyzer and drone beacon (Remote ID) detector, built on an **ESP32-S3** and an **RX5808** module.

The ESP32-S3 does two things in parallel:

- **5.8 GHz spectrum**: it sweeps 5645–5945 MHz with the RX5808 and streams RSSI measurements over the serial port.
- **Drone beacon sniffer**: it listens for the 2.4 GHz Wi-Fi frames that carry drone identification and forwards them as JSON.

A web app (Web Serial) displays the spectrum and a map of detected drones, and can flash the firmware from the browser.

The same firmware can also switch into **RotorHazard Timer Backpack** mode for race timing — see [below](#rotorhazard-timer-backpack-mode). It's a mode switch (one serial command plus a reboot), not a reflash, but the two roles still never run at the same time: they'd both need the one Wi-Fi radio.

## Repository layout

```
firmware/drone_scan_tool/
  drone_scan_tool.ino   Arduino firmware (ESP32-S3) — scanner/sniffer + mode switch
  config.h              Pins, scan range, sniffer settings
  TimerBackpack.h/.cpp  RotorHazard Timer Backpack mode (ESP-NOW <-> serial MSP bridge)
  bp_config.h/.cpp      Timer backpack settings storage (EEPROM)
  msp.h/.cpp, msptypes.h, crc.h/.cpp, elrs_eeprom.h/.cpp
                        Small helpers the Timer Backpack mode is built on,
                        adapted from the ExpressLRS Backpack project (GPL-3.0)
web/
  index.html            Web app (single file, no build step)
```

## Hardware

- ESP32-S3 DevKit
- RX5808 receiver module (with RSSI output accessible)

### Wiring

| RX5808        | ESP32-S3          |
|---------------|-------------------|
| CH1 (DATA)    | GPIO 4            |
| CH2 (LE)      | GPIO 5            |
| CH3 (CLK)     | GPIO 6            |
| RSSI          | GPIO 1 (ADC1_CH0) |
| VCC           | 3.3 V             |
| GND           | GND               |

GPIO 19/20 are reserved for USB CDC. ADC2 is incompatible with Wi-Fi, which is why an ADC1 pin is used.

Two more pins are used only by [Timer Backpack mode](#rotorhazard-timer-backpack-mode): a status LED on GPIO 21, and the board's BOOT button on GPIO 0. Both are unused and harmless in scanning mode.

## Flashing the firmware

1. Install ESP32 support in the Arduino IDE (or `arduino-cli`).
2. Open `firmware/drone_scan_tool/drone_scan_tool.ino`.
3. Select board **ESP32S3 Dev Module**, with these board options (Tools menu in the Arduino IDE):
   - **USB CDC On Boot: Disabled** (the default — this board's USB port is wired through a UART bridge chip, not native USB; enabling CDC-on-boot silently redirects `Serial` to the unused native-USB peripheral and nothing comes out)
   - **PSRAM: OPI PSRAM** (this board has 8 MB of embedded octal PSRAM; leaving PSRAM disabled or set to QSPI silently crashes very early in boot, before anything is printed)
   - **Flash Size: 16MB**
4. Upload. **The first time**, or after flashing anything else to this board, also tick **Erase All Flash Before Sketch Upload: Enabled** once — a leftover NVS partition from a different flash layout can crash boot (again silently, before any serial output) the same way a PSRAM mismatch does. Once it's booted cleanly once, you can leave erase disabled for later uploads.

These settings match the **diymore ESP32-S3 DevKitC-1 N16R8** this project was built against; a different ESP32-S3 board may need different Flash Size/PSRAM values — check its exact chip variant (N*R* suffix) if serial output stays silent after flashing.

Settings (pins, frequency range, sniffer Wi-Fi channel, delays) are in `config.h`.

## Web app

Open `web/index.html` in a Chromium-based browser (Chrome, Edge), since Web Serial is not available in Firefox or Safari. Connect the ESP32-S3, pick the baud rate (115200 by default) and connect.

The app loads Leaflet, CryptoJS and esptool-js from CDNs, so it needs an Internet connection.

## Serial protocol

Output, 115200 baud, one message per line:

| Line                                 | Meaning                                |
|--------------------------------------|----------------------------------------|
| `5658:123`                           | RSSI measurement (frequency MHz:value) |
| `EOS`                                | End of a full sweep                    |
| `BOOT:...`                           | Boot information                       |
| `ACK:...`                            | Command acknowledgement                |
| `DIAG:...`                           | RSSI diagnostic                        |
| `{"id":...}`                         | Drone beacon detected or updated       |
| `{"event":"timeout","id":"..."}`     | Beacon expired (30 s without a frame)  |

Supported commands:

| Command         | Effect                                 |
|-----------------|----------------------------------------|
| `STEP:1/2/5`    | Scan resolution in MHz                 |
| `SCALE:min,max` | Y-axis scale                           |
| `RANGE:...`     | Sweep range                            |
| `CAL:...`       | Calibration                            |
| `FOCUS:XXXX`    | Focus mode on a band                   |
| `FOCUS:OFF`     | Back to normal scan                    |
| `DIAG`          | RSSI diagnostic                        |
| `BACKPACK:PHRASE:<text>` | Set the Timer Backpack binding phrase (see below) |
| `BACKPACK:PHRASE?` | Read back the binding phrase stored on this ESP |
| `MODE:BACKPACK` | Switch into Timer Backpack mode and reboot |

## Supported drone protocols

- DGAC France (order of 27/12/2019, OUI `6A:5C:35`)
- EU Remote ID ASTM F3411-22a (OUI `FA:0B:BC`, subtype `0x0D`)
- EU Remote ID ASTM F3411-19, legacy version (OUI `5F:04:01`)

> ⚠️ **EU Remote ID detection is not reliable yet.** Some beacons may be missed or decoded incorrectly.

The sniffer listens on a single Wi-Fi channel at a time (channel 6 by default, configurable via `WIFI_SNIFFER_CHANNEL`), so it only sees beacons transmitted on that channel.

## RotorHazard Timer Backpack mode

This firmware can also act as a **RotorHazard Timer Backpack**: a USB-serial ⇄ ESP-NOW bridge between RotorHazard and the pilots' VRx backpacks, used for race timing. In that role it:

- Connects to the RotorHazard server over USB serial at 460800 baud, speaking the ExpressLRS [MSP backpack protocol](https://github.com/ExpressLRS/Backpack) (RotorHazard's [VRxC_ELRS plugin](https://github.com/i-am-grub/VRxC_ELRS), RotorHazard v4.1.0+).
- Talks to pilots' VRx backpacks over ESP-NOW to show race stage, start, lap results, positions and gap times on their goggles' OSD. Currently only HDZero goggles with a built-in backpack are supported upstream.
- Forwards the race director's `DVR Rec` switch (start/stop race) from the transmitter's backpack to RotorHazard.

It's **adapted from the official [ExpressLRS Backpack](https://github.com/ExpressLRS/Backpack) firmware** (`src/Timer_main.cpp` and its small device/LED/button/MSP helpers), trimmed to just this bridge — the Wi-Fi OTA update/captive-portal path and the ESP8266 targets were dropped, since this is ESP32-S3-only and already has a USB flashing workflow. That's also why this project is GPL-3.0, not MIT: see [LICENSE](LICENSE).

It's a **mode switch, not a reflash**, and it reuses the one Wi-Fi radio, so it never runs at the same time as the spectrum scan / drone sniffer.

### Auto-detection

The firmware recognises the first few bytes sent by whatever just connected and can switch mode on its own — but this only works both ways if the other end happens to talk at the *current* mode's baud, since this board is a real UART (not native USB), and a baud mismatch garbles the bytes before any detection logic ever sees them:

- **In backpack mode (460800 baud)**, the web app sends a dedicated `SCANAPP:HELLO` line the moment you hit CONNECTER — a deliberate, unambiguous marker, and the *only* thing (besides the explicit `MODE:SCAN` command) that triggers a switch back to scan mode. It used to also guess from other command prefixes as a "secondary check", but that was dropped: with RotorHazard actually connected, there's no way to know its exact traffic won't coincidentally match one of those prefixes, and an unwanted switch back to scan mid-session is a worse failure mode than requiring the one deliberate marker. Since the web app always connects at the baud *you* select, this reliably switches a board left in backpack mode back to scanning, and `connect()` retries the other baud automatically if the first guess doesn't answer. **This direction works plug-and-play.**
- **In scan mode (115200 baud)**, any byte starting an MSP frame (`$`, never sent by our own text protocol) switches straight to backpack mode — but only if it actually arrives at 115200. **RotorHazard always talks at 460800** (fixed by the ExpressLRS protocol), so in practice this direction won't trigger from a real RotorHazard connection: its bytes arrive garbled at the wrong baud, same as any other 460800↔115200 mismatch on this hardware.

In practice, that means: switch to backpack mode explicitly, from the web app (`MODE:BACKPACK`, or the Timer Backpack panel's button), each time before connecting it to RotorHazard. The mode is remembered in RTC memory, not flash: it survives the reboot that `MODE:BACKPACK`/`MODE:SCAN` themselves trigger, but **not a real power cycle** — unplugging the board always brings it back up in scan mode, by design, so you always know what you'll get without having to check. (This also keeps mode-switching off the same small flash-backed NVS storage the bind phrase uses, which doesn't take well to being written often — see the note below.)

### Pairing

Real ExpressLRS/RotorHazard backpacks pair by sharing a **binding phrase**: both ends hash it into the same ESP-NOW group address. This firmware computes that hash the same way the ExpressLRS Configurator does, so it pairs with any officially-built backpack (e.g. HDZero goggles) given the same phrase:

1. While still in scan mode, send `BACKPACK:PHRASE:<your phrase>` over serial. The phrase itself is stored on the ESP (not just the address derived from it), so it survives reboots and can be read back later with `BACKPACK:PHRASE?` — the device replies to either with `ACK:PHRASE=<phrase>,UID=<12 hex digits>` (or just `ACK:PHRASE=<phrase>` for a plain read), so you can double-check both ends match. The web app's Timer Backpack panel does this automatically: it asks the ESP for its stored phrase every time you open the panel, rather than relying on anything remembered in the browser.
2. Send `MODE:BACKPACK` to switch over (reboots at 460800 baud).

Alternatively, pair interactively without knowing a phrase: put both devices into binding mode (send `BACKPACK:BIND` over serial, or the `MSP_ELRS_BACKPACK_SET_MODE 'B'` command from RotorHazard) and they'll adopt each other's address — the same fallback the official firmware uses when no phrase was pre-flashed. The device replies `ACK:BIND=OK,UID=...` once paired, or `ACK:BIND=TIMEOUT` after 2 minutes with no peer.

### Controls in this mode

Everything is reachable over serial alone, no physical button needed:

| Action                          | Effect                                   |
|----------------------------------|-------------------------------------------|
| Send `BACKPACK:BIND` over serial | Enter binding mode                        |
| Send `MODE:SCAN` over serial     | Switch back to scan mode and reboot       |
| Status LED fast blink            | Binding mode (if you have visual access to it) |
| Status LED brief blip            | ESP-NOW activity                          |

If the board's BOOT button (GPIO 0) happens to be reachable, a short press does the same as `BACKPACK:BIND`, and holding it 1.5 s does the same as `MODE:SCAN` — purely a convenience, never required.

The RX5808 wiring is unused in this mode — leaving it connected is harmless.

> Note: the bind phrase and group address are the only things still persisted to flash in this mode (via the ESP32 Arduino core's `EEPROM` library, which stores them in an NVS namespace — no separate `eeprom` partition needed). That NVS partition is small (20 KB) and seems to corrupt if written to very often across many mode switches, going silent on both bauds until the flash is fully erased and reflashed (`Erase All Flash Before Sketch Upload: Enabled`, once). If serial output ever goes completely silent after a lot of back-and-forth testing, that's the fix.

## Disclaimer

This tool is passive: it only listens. Check your local regulations on receiving and using this data before using it.

## License

GPL-3.0-only — see [LICENSE](LICENSE). (Changed from MIT when [Timer Backpack mode](#rotorhazard-timer-backpack-mode) brought in GPL-3.0 code adapted from the ExpressLRS Backpack project.)
