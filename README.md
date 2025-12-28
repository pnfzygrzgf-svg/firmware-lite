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
Dieser Branch nutzt anstelle der Ultraschall-Sensoren ein [TF-Luna](https://en.benewake.com/TFLuna/).

## Building

Required: 
- recent python version installed
- possibly drivers for usb-serial installed

To build and upload to obs-lite do the following:
 
> [!NOTE]
> Some ESP modules will require you to push the boot button at the beginning of the flashing process. Try that in case 
> you get error messages about connection problems while flashing.

```bash
# take care to clone including submodules
git clone --recurse-submodules https://github.com/opatut/obs-firmware-lite.git
cd obs-firmware-lite/

# create a venv to not pollute the global interpreter
python3 -m venv venv

# install platformio
venv/bin/pip install platformio

# build and upload to obs lite
venv/bin/platformio run -t upload
```

## Quickstart

* Install PlatformIO, either the IDE version or just the CLI tools are fine. 
* Wire up the sensors and buttons according to the schematics from [the obs-lite branch of the PCB repository](https://github.com/openbikesensor/OpenBikeSensor_PCB_Board/tree/obs-lite/OpenBikeSensorLite). That is:
    * Sensor A Echo: GPIO 4 / Pin 20
    * Sensor A Trigger: GPIO 15 / Pin 18
    * Sensor B Echo: GPIO 26 / Pin 8
    * Sensor B Trigger: GPIO 25 / Pin 9
    * Button: GPIO 2 / Pin 19 -- button connects the pin to ground (internal pull-up)
* Attach your ESP32 to the USB Port of your computer.
* Build and run the PlatformIO project. It should upload the binary, then start running.
* You should be able to decode the data stream on the serial port using the [OpenBikeSensor Recording Format](https://github.com/openbikesensor/proto), for example with the included [`examples/reader.py`](https://github.com/openbikesensor/proto/blob/main/examples/reader.py) script.
