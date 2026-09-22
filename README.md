# Drone Scan Tool

A 5.8 GHz (FPV) spectrum analyzer and drone beacon (Remote ID) detector, built on an **ESP32-S3** and an **RX5808** module.

The ESP32-S3 does two things in parallel:

- **5.8 GHz spectrum**: it sweeps 5645–5945 MHz with the RX5808 and streams RSSI measurements over the serial port.
- **Drone beacon sniffer**: it listens for the 2.4 GHz Wi-Fi frames that carry drone identification and forwards them as JSON.

A web app (Web Serial) displays the spectrum and a map of detected drones, and can flash the firmware from the browser.

## Repository layout

```
firmware/drone_scan_tool/
  drone_scan_tool.ino   Arduino firmware (ESP32-S3)
  config.h              Pins, scan range, sniffer settings
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

## Flashing the firmware

1. Install ESP32 support in the Arduino IDE (or `arduino-cli`).
2. Open `firmware/drone_scan_tool/drone_scan_tool.ino`.
3. Select the ESP32-S3 board (with USB CDC on boot enabled) and upload.

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

## Supported drone protocols

- DGAC France (order of 27/12/2019, OUI `6A:5C:35`)
- EU Remote ID ASTM F3411-22a (OUI `FA:0B:BC`, subtype `0x0D`)
- EU Remote ID ASTM F3411-19, legacy version (OUI `5F:04:01`)

> ⚠️ **EU Remote ID detection is not reliable yet.** Some beacons may be missed or decoded incorrectly.

The sniffer listens on a single Wi-Fi channel at a time (channel 6 by default, configurable via `WIFI_SNIFFER_CHANNEL`), so it only sees beacons transmitted on that channel.

## Disclaimer

This tool is passive: it only listens. Check your local regulations on receiving and using this data before using it.

## License

MIT — see [LICENSE](LICENSE).
