# Hardware Construction

This page documents the current pin allocation and the ECU targets the firmware has been developed for.

## Board-level layout

The system is split into three main hardware interfaces:

- GPS receiver on `uart0`
- LR1121 LoRa radio on a dedicated SPI bus
- MCP2515 CAN controller on a separate SPI bus

That separation keeps the radio, CAN, and GPS paths independent at the wiring level.

## Pin allocation

### GPS

| Signal | GPIO |
| --- | --- |
| GPS TX | GPIO 0 |
| GPS RX | GPIO 1 |

The firmware configures these pins as `uart0` in [`gps.c`](/home/louis/Documents/FS26_DAQ/FS26-DAQ/gps.c#L206).

### LR1121 radio

The LR1121 support layer binds the radio context to these pins:

| Signal | GPIO |
| --- | --- |
| RESET | GPIO 8 |
| BUSY | GPIO 9 |
| CLK | GPIO 10 |
| MOSI | GPIO 11 |
| MISO | GPIO 12 |
| CS | GPIO 13 |
| IRQ | GPIO 14 |
| LED | GPIO 25 |

These definitions live in [`src/gpio/gpio.h`](/home/louis/Documents/FS26_DAQ/FS26-DAQ/src/gpio/gpio.h#L20) and are consumed by [`src/lr1121/wavesahre_lora_1121.c`](/home/louis/Documents/FS26_DAQ/FS26-DAQ/src/lr1121/wavesahre_lora_1121.c#L1).

The radio SPI helper currently initializes `spi1` for the LR1121 path in [`src/spi/spi.h`](/home/louis/Documents/FS26_DAQ/FS26-DAQ/src/spi/spi.h#L1).

### MCP2515 CAN controller

The CAN interface uses the board-support pin map defined in the MCP2515 config layer:

| Signal | GPIO |
| --- | --- |
| SPI SCK | GPIO 6 |
| SPI MOSI | GPIO 7 |
| SPI MISO | GPIO 4 |
| CS0 | GPIO 5 |
| CS1 | GPIO 1 |

The active chip select is `MCP2515_CS0_PIN` / `GPIO 5`, as defined in [`src/mcp2515/Config/DEV_Config.h`](/home/louis/Documents/FS26_DAQ/FS26-DAQ/src/mcp2515/Config/DEV_Config.h#L45).
The CAN support layer initializes `spi0` in [`src/mcp2515/Config/DEV_Config.c`](/home/louis/Documents/FS26_DAQ/FS26-DAQ/src/mcp2515/Config/DEV_Config.c#L1).

## ECU targets

### FT550

The repository still contains a full FT550 decoder in [`ft550_decoder.c`](/home/louis/Documents/FS26_DAQ/FS26-DAQ/ft550_decoder.c#L1) and [`ft550_decoder.h`](/home/louis/Documents/FS26_DAQ/FS26-DAQ/ft550_decoder.h#L1).

That decoder is structured around extended 29-bit frame IDs `0x14080600` through `0x14080608` and covers sensor groups such as:

- TPS and MAP
- temperatures and pressures
- RPM and wheel speeds
- traction, shocks, g-forces, and brake pressure

This is the clearest reference for the original FT550-style telemetry layout.

### M84

The live CAN handler in [`can_handler.c`](/home/louis/Documents/FS26_DAQ/FS26-DAQ/can_handler.c#L45) is currently reworked for MoTeC M84 decoding.

In the active implementation, the firmware:

- receives a burst of CAN frames on identifier `0x100`
- assembles them into a temporary block
- searches for the M84 magic sequence `82 81 80 54`
- extracts values such as TPS, RPM, engine temperature, air temperature, battery voltage, and MAP from offsets relative to that anchor

So the codebase currently supports both:

- the FT550 frame decoder as a reusable library
- the M84 burst decoder as the live CAN path

## Practical notes

- GPS uses a separate UART, so it does not share pins with the radio or CAN bus.
- The LR1121 and MCP2515 each have their own SPI wiring and chip select pins.
- The current firmware treats the CAN source as a burst-based ECU stream rather than a simple one-frame-per-sensor layout.
