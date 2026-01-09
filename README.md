# OpenBikeSensor Lite Firmware

This is the firmware repository for the up-and-coming "Lite" version of the OpenBikeSensor.

If you are looking for the firmware for the existing, "Classic" OpenBikeSensor, you can find it here: https://github.com/openbikesensor/OpenBikeSensorFirmware

The OpenBikeSensor Lite is currently in development and not available to build yet. There are no build instructions, and this firmware, the recording software, data format and enclosure are not completed. Please go over to the [OpenBikeSensor Website](https://www.openbikesensor.org/docs/hardware/) if you're interested in building your own device, it contains all information for the Classic version of the OpenBikeSensor.

## BLE-Datenstream (dieser Fork)

Dieser Fork streamt OpenBikeSensor-Events über **Bluetooth Low Energy (BLE)** mittels GATT-Notifications.

### BLE-Gerät
- Advertising-Name: `OpenBikeSensor`
- Service UUID: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- TX-Characteristic UUID (Notify): `6e400003-b5a3-f393-e0a9-e50e24dcca9e`

## Lidar (dieser Branch)
Dieser Branch nutzt anstelle der Ultraschall-Sensoren zwei [TF-Luna](https://en.benewake.com/TFLuna/).

## Building

Required: 
- recent python version installed
- possibly drivers for usb-serial installed

To build and upload to obs-lite do the following:
 
> [!NOTE]
> Some ESP modules will require you to push the boot button at the beginning of the flashing process. Try that in case 
> you get error messages about connection problems while flashing.

```bash
# Fork klonen (inkl. Submodules) und den gewünschten Branch auschecken
git clone --recurse-submodules -b firmware-lite-2-lidar \
  https://github.com/pnfzygrzgf-svg/firmware-lite.git
cd firmware-lite/

# Virtuelle Umgebung erstellen, um die globale Python-Installation nicht zu verändern
python3 -m venv venv

# PlatformIO installieren
venv/bin/pip install platformio

# Firmware bauen und auf den OBS-Lite flashen
venv/bin/platformio run -t upload
```
* Attach your ESP32 to the USB Port of your computer.
* Build and run the PlatformIO project. It should upload the binary, then start running.
* You should be able to decode the data stream on the serial port using the [OpenBikeSensor Recording Format](https://github.com/openbikesensor/proto), for example with the included [`examples/reader.py`](https://github.com/openbikesensor/proto/blob/main/examples/reader.py) script.
