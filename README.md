[Deutsch](#openbikesensor-lite-firmware-fork) | [English](#openbikesensor-lite-firmware-fork-1)

---

# OpenBikeSensor Lite Firmware (Fork)

> **Hinweis:** Dieses Repository ist ein **Fork** der OpenBikeSensor Lite Firmware und enthält Erweiterungen
> (u.a. BLE-Streaming von OpenBikeSensor-Events).
>
> Wenn du die Firmware für den bestehenden „Classic" OpenBikeSensor suchst, findest du sie hier:  
> https://github.com/openbikesensor/OpenBikeSensorFirmware

> [!NOTE]
> Upstream („Lite") ist/war in Entwicklung. Dieser Fork ist experimentell und enthält eigene Build-Anweisungen.

---

## BLE-Datenstream (dieser Fork)

Dieser Fork streamt OpenBikeSensor-Events über **Bluetooth Low Energy (BLE)** mittels GATT-Notifications
(Nordic UART Style).

### BLE-Gerät

| Eigenschaft | Wert |
|---|---|
| Advertising-Name | `OpenBikeSensor` |
| Service UUID | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` |
| TX-Characteristic UUID (Notify) | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` |

---

## Building (PlatformIO)

Voraussetzungen:
- Python (aktuell)
- ggf. USB-Serial Treiber (abhängig vom USB-UART auf deinem ESP32-Board)

> [!NOTE]
> Manche ESP-Module erfordern, dass du zu Beginn des Flashvorgangs den **BOOT**-Button drückst.
> Probiere das, falls du Fehlermeldungen zu Verbindungsproblemen beim Flashen bekommst.
>
> Bei instabilen Uploads kannst du `upload_speed` in `platformio.ini` reduzieren (z. B. `460800` oder `115200`).

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

---

# OpenBikeSensor Lite Firmware (Fork)

> **Note:** This repository is a **fork** of the OpenBikeSensor Lite Firmware and contains additional features
> (including BLE streaming of OpenBikeSensor events).
>
> If you are looking for the firmware for the original "Classic" OpenBikeSensor, you can find it here:  
> https://github.com/openbikesensor/OpenBikeSensorFirmware

> [!NOTE]
> The upstream ("Lite") project is/was under active development. This fork is experimental and includes its own build instructions.

---

## BLE Data Stream (this fork)

This fork streams OpenBikeSensor events via **Bluetooth Low Energy (BLE)** using GATT notifications
(Nordic UART style).

### BLE Device

| Property | Value |
|---|---|
| Advertising Name | `OpenBikeSensor` |
| Service UUID | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` |
| TX Characteristic UUID (Notify) | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` |

---

## Building (PlatformIO)

Requirements:
- Python (current version)
- USB serial driver if needed (depends on the USB-UART chip on your ESP32 board)

> [!NOTE]
> Some ESP modules will require you to push the **BOOT** button at the beginning of the flashing process.
> Try that in case you get error messages about connection problems while flashing.
>
> If uploads are unstable on your setup, reduce `upload_speed` in `platformio.ini` (e.g. `460800` or `115200`).

```bash
# Clone the fork (including submodules)
git clone --recurse-submodules https://github.com/pnfzygrzgf-svg/firmware-lite.git
cd firmware-lite/

# Create a virtual environment
python3 -m venv venv

# Install PlatformIO
venv/bin/pip install platformio

# Build and flash the firmware to the OBS Lite
venv/bin/platformio run -t upload
```
