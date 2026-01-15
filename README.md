# OpenBikeSensor Lite Firmware (Fork)

> **Hinweis:** Dieses Repository ist ein **Fork** der OpenBikeSensor Lite Firmware und enthält Erweiterungen
> (u.a. BLE-Streaming von OpenBikeSensor-Events).
>
> Wenn du die Firmware für den bestehenden „Classic“ OpenBikeSensor suchst, findest du sie hier:  
> https://github.com/openbikesensor/OpenBikeSensorFirmware

> [!NOTE]
> Upstream („Lite“) ist/war in Entwicklung. Dieser Fork ist experimentell und enthält eigene Build-Anweisungen.

---

## BLE-Datenstream (dieser Fork)

Dieser Fork streamt OpenBikeSensor-Events über **Bluetooth Low Energy (BLE)** mittels GATT-Notifications
(Nordic UART Style).

### BLE-Gerät
- Advertising-Name: `OpenBikeSensor`
- Service UUID: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- TX-Characteristic UUID (Notify): `6e400003-b5a3-f393-e0a9-e50e24dcca9e`

---

## Building (PlatformIO)

Required:
- Python (aktuell)
- ggf. USB-Serial Treiber (abhängig vom USB-UART auf deinem ESP32-Board)

> [!NOTE]
> Some ESP modules will require you to push the BOOT button at the beginning of the flashing process.
> Try that in case you get error messages about connection problems while flashing.
>
> If uploads are unstable on your setup, reduce `upload_speed` in `platformio.ini` (e.g. `460800` or `115200`).

```bash
# Fork klonen (inkl. Submodules)
git clone --recurse-submodules https://github.com/pnfzygrzgf-svg/firmware-lite.git
cd firmware-lite/

# Virtuelle Umgebung erstellen
python3 -m venv venv

# PlatformIO installieren
venv/bin/pip install platformio

# Firmware bauen und auf den OBS-Lite flashen
venv/bin/platformio run -t upload
```