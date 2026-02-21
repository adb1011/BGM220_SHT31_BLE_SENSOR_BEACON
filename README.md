# SHT31 RHT Beacon

This repository contains firmware for a Bluetooth Low Energy (BLE) environmental beacon based on the 
Silicon Labs Bluetooth stack and an SHT31 temperature/humidity sensor. The device periodically reads 
environmental data and advertises it as a custom beacon payload.

## Features
- Periodic sensor measurements from SHT31 (temperature & humidity)
- Encodes sensor data in manufacturer-specific advertising packet
- 100 ms BLE advertising interval (non-connectable beacon mode)
- Optional button to trigger an immediate sensor update
- Configurable update interval (default 1 s)
- Development-friendly settings and logging via `app_log` macros

## Building
Use Simplicity Studio or the provided CMake build system:
```sh
cd sht31_rht_beacon
mkdir -p build && cd build
cmake -G "Unix Makefiles" ..
make
```
Flash the resulting firmware using the Simplicity Commander or IDE.

## Configuration
- I2C address for the SHT31 can be changed in `sht31.h` if needed.
- Advertising parameters and sensor update interval are defined in `app.c`.

## File overview
- `app.c` – main application logic and BLE event handler
- `sht31.c/h` – sensor driver for SHT31
