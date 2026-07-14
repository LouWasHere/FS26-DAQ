# Telemetry Flow

This page describes the live telemetry path through the firmware.

## GPS

`gps_init()` starts the GPS module at 9600 baud, checks whether the receiver is already running at 9600 or 57600, then configures:

- 5 Hz update rate
- selected NMEA output sentences
- 57600 baud for normal operation

`gps_process()` buffers incoming NMEA characters, verifies the checksum, and parses:

- GGA for latitude, longitude, altitude, satellites, and HDOP
- RMC for speed and course

The GPS module also applies simple filtering:

- discard low-quality fixes when HDOP is too high
- suppress coordinate drift while the vehicle is stationary

## CAN / FT550

`can_init()` configures the MCP2515 for 1 Mbps extended CAN traffic.

The live handler currently assembles received frames into a block, searches for the MoTeC/FT550 magic number, and extracts selected values such as:

- TPS
- RPM
- engine temperature
- air temperature
- battery voltage
- MAP

The decoder library in `ft550_decoder.c` supports the full FT550 frame map and can be used for more direct per-frame parsing.

## LoRa packet

`FS26-DAQ.c` builds a packed telemetry structure containing:

- magic value `0x46533236` (`FS26`)
- GPS position, speed, altitude, satellite count, and fix state
- key CAN values such as RPM, engine temp, throttle, pressures, wheel speeds, and heading
- metadata such as LoRa TX count and CAN frame count

The packet is sent by `lora_send()` from core 1.

## Dashboard CAN output

The main loop also publishes a compact set of dashboard frames on the local CAN bus:

- `0x600` primary engine values
- `0x601` battery and air temperature
- `0x602` GPS position
- `0x603` GPS and telemetry metadata
