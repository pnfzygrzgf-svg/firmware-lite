# OpenBikeSensor Lite Firmware (Fork)

> **Hinweis:** Dieses Repository ist ein **Fork** der OpenBikeSensor Lite Firmware und enthält Erweiterungen.  
> Dieser Branch (**`firmware-lite-lidar`**) streamt OpenBikeSensor Events über **BLE** und zusätzlich über  
> **USB-Serial** (PacketSerial/SLIP framing).

Wenn du die Firmware für den bestehenden „Classic“ OpenBikeSensor suchst, findest du sie hier:  
https://github.com/openbikesensor/OpenBikeSensorFirmware

---

## Features in diesem Fork/Branch

### BLE-Datenstream
OpenBikeSensor-Events werden über **Bluetooth Low Energy (BLE)** via GATT-Notifications im Nordic-UART-Stil gesendet.

- Advertising-Name: `OBS Lite LiDAR`
- Service UUID: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- TX-Characteristic UUID (Notify): `6e400003-b5a3-f393-e0a9-e50e24dcca9e`

### USB-Serial Datenstream
Zusätzlich werden die Events über **USB-Serial** ausgegeben (PacketSerial / SLIP framing), damit Hosts
(z.B. Android oder Desktop) direkt per Kabel mitlesen können.

---

## Build & Flash (PlatformIO)

Voraussetzungen:
- Python (aktuell)
- ggf. USB-Serial Treiber (abhängig vom USB-UART auf deinem ESP32-Board)

> [!NOTE]
> Manche ESP32-Boards benötigen beim Flashen den **BOOT**-Button (gedrückt halten, kurz Reset/EN drücken).  
> Falls Upload-Probleme auftreten: `upload_speed` in `platformio.ini` reduzieren (z.B. `460800` oder `115200`).

```bash
# Fork klonen (inkl. Submodules) und diesen Branch auschecken
git clone --recurse-submodules -b firmware-lite-lidar \
  https://github.com/pnfzygrzgf-svg/firmware-lite.git
cd firmware-lite/

# Virtuelle Umgebung erstellen
python3 -m venv venv

# PlatformIO installieren
venv/bin/pip install platformio

# Firmware bauen und flashen
venv/bin/platformio run -t upload
```