# **Accelerometer Read Application - Polling**

## Intent

The application was intended for the Adafruit nRF52840 Feather Sense dev board,
but is easily portable to other devices. This is a very basic implementation
of an application that polls a sensor at predetermined intervals.

## Building the code

Clone the repo and grab the submodules:

```[bash]
git clone --recurse-submodules git@github.com:jacobw56/accel_read_poll.git
```

Assuming you have a valid GNU Arm Embedded Toolchain for 32-bit bare metal
targets, you can just use `cmake` as usual, but with the toolchain file and
local variables `ARM_NONE_EABI_PATH` and `SDK_ROOT_DIR` appropiately defined:

```[bash]
cd accel_read_poll
mkdir build
cd build
cmake \
    -DARM_NONE_EABI_PATH=${HOME}/.local/arm-none-eabi \
    -DSDK_ROOT_DIR=${PWD}/../nrf_sdk \
    -DCMAKE_TOOLCHAIN_FILE=${PWD}/../toolchain.cmake \
    ..
make
```

Optionally you can also define `DEBUG` to build `-Og -g3` or `LTO` to build
`-flto`.

**_NOTA BENE_**
This firmware is built on the nRF5 SDK, version 17.0.2 (d674dde), **_however_**
it requires a version of nrfx_spim.c that differs from the original. You can
find this modified SDK in the nrf_sdk directory.
