# Thingy:53 firmware

[Edge Impulse](https://www.edgeimpulse.com) enables developers to create the next generation of intelligent device solutions with embedded Machine Learning. This repository contains the Edge Impulse firmware for the Nordic Semiconductor Thingy:53 development board that includes many different sensors. This firmware supports all Edge Impulse features, including ingestion, remote management, and inferencing. Direct communication with the device is performed over a mobile application, but the user can execute all functions via AT command over a USB interface. 

## Requirements

**Hardware**

* Nordic Semiconductor Thingy:53
* nRF Connect SDK 1.9.1

**Software**

* [nRF Connect SDK 1.9.1](https://www.nordicsemi.com/Software-and-tools/Software/nRF-Connect-SDK)
* [GNU ARM Embedded Toolchain 9-2019-q4-major](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads).
* [nRF Command Line Tools](https://www.nordicsemi.com/Software-and-tools/Development-Tools/nRF-Command-Line-Tools/Download).
* [CMake v3.21.3](https://cmake.org/download/)

Or you can build this application with Docker (see below).

## Building the device firmware (locally)

1. Clone this repository:

    ```
    $ git clone https://github.com/edgeimpulse/firmware-nordic-thingy53-internal
    ```

2. Build the application:

    ```
    $ west build -b thingy53_nrf5340_cpuapp
    ```

    > When doing DFU over nRF Connect application, `app_update.bin` file needs to be used (this file is also in the `dfu_application.zip`).

## Building the device firmware (Docker)

1. Clone this repository:

    ```
    $ git clone https://github.com/edgeimpulse/firmware-nordic-thingy53-internal
    ```

1. Build the Docker container:

    ```
    $ docker build -t edge-impulse-nordic .
    ```

1. Build the application:

    ```
    $ docker run --rm -v $PWD:/app edge-impulse-nordic west build -b thingy53_nrf5340_cpuapp
    ```

1. Copy `build/zephyr/zephyr.bin` to the `JLINK` mass storage device.

## Flashing

### Over JLink

1. Connect the JLink to the Thingy:53; if not previously done, erase the chip with:

    ```
    nrfjprog --recover
    ```  

1. Flash the chip, both application core and networking core, with the command: 

    ```
    west flash
    ```

### Over Serial DFU

1. Connect USB-C to the Thingy:53. Open the enclosure, press the small button (`SW2`) on top of the device and turn the power switch on.

    ![Recovery button location](./docs/recovery-button.png)

1. After the device is on, release the button. The device is now in bootloader mode. For flashing the new firmware, use commands:

    ```
    mcumgr -t 60 --conntype serial --connstring=/dev/ttyACM0 image list
    mcumgr -t 60 --conntype serial --connstring=/dev/ttyACM0 image upload app_update.bin
    ```

### Over BLE DFU

#### Use mobile app (Android/iOS)

Use [nRF Connect for Android](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp) and `build/zephyr/app_update.bin`. After connecting to the device, click on the DFU icon and select the file. When the DFU process is done, the device will boot with the new image. 

#### Use `mcumgr`

1. Install [mcumgr](https://docs.zephyrproject.org/latest/guides/device_mgmt/mcumgr.html)
1. Upload the firmware (adjust the path to `app_update.bin` and `hci` interface number if you have a few of them)
    ```
    mcumgr --conntype ble --hci 0 --connstring peer_name='EdgeImpulse' image upload app_update.bin
    ```
1. Check if the firmware has been uploaded correctly and get the hash of new firmware
    ```
    mcumgr --conntype ble --hci 0 --connstring peer_name='EdgeImpulse' image list
    ```
    The output should be similar to the one below; the new image is in slot 1.
    ```
    Images:
    image=0 slot=0
        version: 0.9.9
        bootable: true
        flags: active confirmed
        hash: 67f6f87f3f639217140f9e8073cb28f214499d7f646cb774276496108326a7ba
    image=0 slot=1
        version: 0.9.1
        bootable: true
        flags: 
        hash: c218220606d3ecef26dc46ffd7c112e700d08837335516c40ae12f6a86aa7ab2
    Split status: N/A (0)
    ```
1. Test the image and swap the images
    ```
    mcumgr --conntype ble --hci 0 --connstring peer_name='EdgeImpulse' image test <HASH_OF_THE_IMAGE>
    ```
1. Reset the device (may take up to 10-20 seconds)
    ```
    mcumgr --conntype ble --hci 0 --connstring peer_name='EdgeImpulse' reset
    ```

## Creating a production firmware

`MCUboot` and `b0n` bootloaders in NCS >1.7.0 have a bug resulting in a network core failed after full OTA update. Therefore (up to NCS 1.9.1) the full firmware has to be made as a mix of components from NCS 1.7.0 and 1.9.1

1. Get NCS 1.7.0 (see instruction above) and build the following example

    ```
    export ZEPHYR_BASE=~/ncs-1.7.0/zephyr/
    cd ~/ncs-1.7.0/nrf/samples/bluetooth/peripheral_uart/
    west build -b thingy53_nrf5340_cpuapp
    ```

1. Build firmware from this repository (see instruction above)
1. Merge required components (run from this repository level)

    ```
    ${ZEPHYR_BASE}/scripts/mergehex.py -o merged_CPUNET.hex --overlap replace ~/ncs-1.7.0/nrf/samples/bluetooth/peripheral_uart/build/hci_rpmsg/zephyr/b0n_container.hex build/hci_rpmsg/zephyr/signed_by_b0_app.hex

    ${ZEPHYR_BASE}/scripts/mergehex.py -o merged_CPUAPP.hex --overlap replace ~/ncs-1.7.0/nrf/samples/bluetooth/peripheral_uart/build/mcuboot/zephyr/zephyr.hex build/zephyr/app_signed.hex
    ```
