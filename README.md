# Drone Scan Tool

Analyseur de spectre 5,8 GHz (FPV) et détecteur de balises drone (Remote ID), basé sur un **ESP32-S3** et un module **RX5808**.

L'ESP32-S3 fait deux choses en parallèle :

- **Spectre 5,8 GHz** : il balaie 5645–5945 MHz avec le RX5808 et envoie les mesures RSSI par le port série.
- **Sniffer de balises drone** : il écoute les trames Wi-Fi 2.4 GHz qui transportent l'identification des drones et les renvoie en JSON.

Une application web (Web Serial) affiche le spectre et la carte des drones détectés, et permet de flasher le firmware depuis le navigateur.

## Contenu du dépôt

```
firmware/drone_scan_tool/
  drone_scan_tool.ino   Firmware Arduino (ESP32-S3)
  config.h              Broches, plage de scan, paramètres du sniffer
web/
  index.html            Application web (fichier unique, sans build)
```

## Matériel

- ESP32-S3 DevKit
- Module récepteur RX5808 (avec RSSI accessible)

### Branchements

| RX5808        | ESP32-S3         |
|---------------|------------------|
| CH1 (DATA)    | GPIO 4           |
| CH2 (LE)      | GPIO 5           |
| CH3 (CLK)     | GPIO 6           |
| RSSI          | GPIO 1 (ADC1_CH0)|
| VCC           | 3.3 V            |
| GND           | GND              |

GPIO 19/20 sont réservés à l'USB CDC. L'ADC2 est incompatible avec le Wi-Fi, d'où l'usage d'une broche ADC1.

## Installation du firmware

1. Installer le support ESP32 dans l'Arduino IDE (ou `arduino-cli`).
2. Ouvrir `firmware/drone_scan_tool/drone_scan_tool.ino`.
3. Choisir la carte ESP32-S3 (avec USB CDC on boot activé) puis téléverser.

Les réglages (broches, plage de fréquences, canal Wi-Fi du sniffer, délais) sont dans `config.h`.

## Application web

Ouvrir `web/index.html` dans un navigateur basé sur Chromium (Chrome, Edge), car Web Serial n'existe pas dans Firefox ni Safari. Connecter l'ESP32-S3, choisir le débit (115200 par défaut) et se connecter.

L'application charge Leaflet, CryptoJS et esptool-js depuis des CDN, elle a donc besoin d'une connexion Internet.

## Protocole série

Sortie, 115200 bauds, une ligne par message :

| Ligne                                | Signification                          |
|--------------------------------------|----------------------------------------|
| `5658:123`                           | Mesure RSSI (fréquence MHz : valeur)   |
| `EOS`                                | Fin d'un balayage complet              |
| `BOOT:...`                           | Informations au démarrage              |
| `ACK:...`                            | Acquittement d'une commande            |
| `DIAG:...`                           | Diagnostic RSSI                        |
| `{"id":...}`                         | Balise drone détectée ou mise à jour   |
| `{"event":"timeout","id":"..."}`     | Balise expirée (30 s sans trame)       |

Commandes acceptées :

| Commande       | Effet                                  |
|----------------|----------------------------------------|
| `STEP:1/2/5`   | Résolution du scan en MHz              |
| `SCALE:min,max`| Échelle Y                              |
| `RANGE:...`    | Plage de balayage                      |
| `CAL:...`      | Calibration                            |
| `FOCUS:XXXX`   | Mode focus sur une bande               |
| `FOCUS:OFF`    | Retour au scan normal                  |
| `DIAG`         | Diagnostic RSSI                        |

## Protocoles drone reconnus

- DGAC France (arrêté du 27/12/2019, OUI `6A:5C:35`)
- EU Remote ID ASTM F3411-22a (OUI `FA:0B:BC`, subtype `0x0D`)
- EU Remote ID ASTM F3411-19, ancienne version (OUI `5F:04:01`)

Le sniffer écoute un seul canal Wi-Fi à la fois (canal 6 par défaut, modifiable via `WIFI_SNIFFER_CHANNEL`), il ne voit donc que les balises émises sur ce canal.

## Avertissement

Cet outil est passif : il écoute seulement. Vérifiez la réglementation locale sur la réception et l'usage de ces données avant de l'utiliser.
