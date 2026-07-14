# FS26-DAQ Wiki

FS26-DAQ is a dual-core telemetry data-acquisition system for the FS26 Leeds Gryphons platform.
The current firmware combines GPS positioning, CAN telemetry from the FT550 ECU, and LoRa broadcast on the LR1121 radio.

## Wiki Map

- [System Architecture](Architecture.md)
- [Hardware Construction](Hardware-Construction.md)
- [Telemetry Flow](Telemetry-Flow.md)
- [Build and Deploy](Build-and-Deploy.md)

## What the firmware does

- Core 0 handles GPS UART processing and CAN reception.
- Core 1 handles LoRa transmission.
- GPS data is parsed from NMEA sentences, filtered, and shared safely across cores.
- CAN data is collected from the FT550 stream and merged into the transmitted telemetry.

## Where to start in the code

- [`FS26-DAQ.c`](../FS26-DAQ.c) contains the main dual-core control flow.
- [`gps.c`](../gps.c) implements GPS setup, parsing, and filtering.
- [`can_handler.c`](../can_handler.c) handles MCP2515 reception and sensor extraction.
- [`lr1121_tx.c`](../lr1121_tx.c) implements LR1121 TX-only operation.
