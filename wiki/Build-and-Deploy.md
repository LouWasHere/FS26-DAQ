# Build and Deploy

## Prerequisites

The project is built with the Raspberry Pi Pico SDK and targets `PICO_BOARD=pico2`.
The generated build artifacts are stored in `build/`.

## Build

The workspace already defines a VS Code build task named `Compile Project`, which runs Ninja against the `build` directory.

If you want to build manually, the equivalent command is:

```bash
ninja -C build
```

## Flash / run

The workspace also includes task entries for deployment:

- `Run Project` uses `picotool load` with the current ELF
- `Flash` uses OpenOCD to program the board
- `Rescue Reset` and `RISC-V Reset (RP2350)` are recovery helpers

## Useful notes

- `pico_enable_stdio_usb(FS26-DAQ 1)` enables USB serial output.
- `pico_enable_stdio_uart(FS26-DAQ 0)` disables default UART stdio so the GPS UART can stay dedicated.
- `pico_add_extra_outputs(FS26-DAQ)` generates UF2 and other standard Pico build artifacts.
