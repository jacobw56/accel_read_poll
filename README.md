# **Accelerometer Read Application - Polling**

## Intent

The application was intended for the Adafruit nRF52840 Feather Sense dev board,
but is easily portable to other devices. This is a very basic implementation
of an application that polls a sensor at predetermined intervals.

## Building the code

Clone the repo and grab the submodules:

**_NOTA BENE_**
This firmware is built on the nRF5 SDK, version 17.0.2 (d674dde), **_however_**
it requires a version of nrfx_spim.c that differs from the original. You can
find this modified SDK in the nrf_sdk directory.
