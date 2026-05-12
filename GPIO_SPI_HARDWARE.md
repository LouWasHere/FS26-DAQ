# Hardware Abstraction Layer (HAL) - GPIO & SPI

## Overview

The GPIO and SPI modules provide hardware-level interfaces to RP2350 peripherals. These are low-level drivers that support radio control (LR1121), CAN controller (MCP2515), and general-purpose I/O.

---

## GPIO Module

### Overview
Manages GPIO pin configuration, digital I/O operations, and interrupt handling.

**Include File:**
```c
#include "src/gpio/gpio.h"
```

### Pin Definitions

#### LR1121 Radio Pins
```c
#define RADIO_RESET 8       // Output - Reset signal
#define RADIO_MOSI  11      // Output - SPI data (controller → device)
#define RADIO_MISO  12      // Input - SPI data (device → controller)
#define RADIO_CLK   10      // Output - SPI clock
#define RADIO_CS    13      // Output - SPI chip select
#define RADIO_BUSY  9       // Input - Radio busy flag
#define RADIO_IRQ   14      // Input - Interrupt request
#define RADIO_LED   25      // Output - LED indicator
```

### Public Functions

#### `void DEV_GPIO_Mode(uint16_t Pin, uint16_t Mode)`
Configure a GPIO pin as input or output.

**Parameters:**
- `Pin` - GPIO pin number (0-29)
- `Mode` - Pin mode:
  - `0` or `GPIO_MODE_INPUT` - Input (pull-up enabled)
  - Other values - Output (push-pull)

```c
// Configure pin 9 as input (RADIO_BUSY)
DEV_GPIO_Mode(RADIO_BUSY, GPIO_MODE_INPUT);

// Configure pin 13 as output (RADIO_CS)
DEV_GPIO_Mode(RADIO_CS, 1);  // Non-zero = output
```

**Details:**
- Input pins have internal pull-up resistor enabled
- Output pins are configured for push-pull (standard GPIO)
- No pull-down configuration currently supported

---

#### `void DEV_GPIO_INT(int32_t Pin, gpio_irq_callback_t isr_handler)`
Configure a GPIO pin to generate interrupts on falling edge.

**Parameters:**
- `Pin` - GPIO pin number
- `isr_handler` - Callback function signature: `void handler(uint gpio, uint32_t events)`

```c
void radio_irq_handler(uint gpio, uint32_t events) {
    if (gpio == RADIO_IRQ) {
        // Radio interrupt detected
        radio_process_interrupt();
    }
}

// Enable interrupt on RADIO_IRQ (pin 14)
DEV_GPIO_INT(RADIO_IRQ, radio_irq_handler);
```

**Features:**
- Triggers on negative edge (high → low transition)
- Callback invoked from IRQ context
- Ensure callback is very brief (critical section)

**Interrupt Callback Requirements:**
- Keep handler as short as possible
- Set flags rather than doing heavy processing
- Use atomic operations for shared variables
- Don't call blocking functions (sleep, printf with mutex, etc.)

---

#### `void DEV_GPIO_WriteValue(uint16_t Pin, uint16_t Value)`
Set GPIO output pin to high or low.

**Parameters:**
- `Pin` - GPIO pin number
- `Value` - Pin state: 0 (low), non-zero (high)

```c
// Drive radio reset low (active reset)
DEV_GPIO_WriteValue(RADIO_RESET, 0);
sleep_ms(10);

// Release reset (inactive)
DEV_GPIO_WriteValue(RADIO_RESET, 1);
```

---

#### `uint16_t DEV_GPIO_ReadValue(uint16_t Pin)`
Read the digital input level of a GPIO pin.

**Returns:** 0 (low) or 1 (high)

```c
while (DEV_GPIO_ReadValue(RADIO_BUSY)) {
    // Wait for radio to finish operation
    sleep_us(100);
}
printf("Radio ready\n");
```

---

### Common GPIO Patterns

#### Polling for Pin State
```c
// Wait for radio BUSY to clear (active high)
uint32_t timeout = 1000000;  // 1 second in µs
uint32_t start = time_us_32();

while (DEV_GPIO_ReadValue(RADIO_BUSY)) {
    if (time_us_32() - start > timeout) {
        printf("ERROR: Radio BUSY timeout\n");
        return false;
    }
    sleep_us(10);
}
```

#### Toggling LED
```c
// Blink LED at 1 Hz
while (true) {
    DEV_GPIO_WriteValue(RADIO_LED, 1);  // LED on
    sleep_ms(500);
    
    DEV_GPIO_WriteValue(RADIO_LED, 0);  // LED off
    sleep_ms(500);
}
```

---

## SPI Module

### Overview
Manages SPI communication for the LR1121 radio and other SPI devices. Uses RP2350's SPI1 peripheral.

**Include File:**
```c
#include "src/spi/spi.h"
```

### Configuration

```c
#define SPI_PORT spi1  // RP2350 SPI instance 1
```

**Pin Mapping:**
```
GPIO 10 - SCLK (Serial Clock)
GPIO 11 - MOSI (Master Out, Slave In) - TX
GPIO 12 - MISO (Master In, Slave Out) - RX
GPIO 13 - CS   (Chip Select) - Controlled manually
```

### Public Functions

#### `void DEV_SPI_Init(void)`
Initialize SPI1 peripheral.

**Called from:** `lora_tx_init()` during radio startup

```c
void lora_tx_init(void) {
    DEV_SPI_Init();      // Configure SPI
    // ... radio initialization ...
}
```

**Configuration Applied:**
- SPI clock frequency: 1 MHz (configurable in implementation)
- Mode: SPI mode 0 (CPOL=0, CPHA=0)
- Bit order: MSB first
- Data width: 8 bits per transfer

**Side Effects:**
- Configures GPIO 10-13 for SPI function
- Initializes SPI1 controller
- Sets default clock speed

---

#### `void DEV_SPI_Write_Bytes(const uint8_t* tx_buf, size_t length)`
Write data to SPI bus (transmit only).

**Parameters:**
- `tx_buf` - Pointer to data to transmit
- `length` - Number of bytes to send

```c
uint8_t cmd[3] = {0x80, 0x01, 0x02};  // Example SPI command
DEV_SPI_Write_Bytes(cmd, 3);
```

**Notes:**
- Blocking call - waits for transmission to complete
- Does not read return data (use `DEV_SPI_Read_Bytes()` for that)
- Typical speed: 8 bits/µs at 1 MHz clock

---

#### `void DEV_SPI_Read_Bytes(uint8_t* rx_buf, size_t length)`
Read data from SPI bus (receive only).

**Parameters:**
- `rx_buf` - Pointer to buffer for received data
- `length` - Number of bytes to read

```c
uint8_t response[4];
DEV_SPI_Read_Bytes(response, 4);
```

**Notes:**
- Blocking call - waits for all bytes to be received
- Transmits zeros (0x00) while reading
- Typical speed: 8 bits/µs at 1 MHz clock

---

### SPI Communication Pattern

#### Full-Duplex Transfer (Write + Read)
```c
// Send command and read response in one atomic operation
uint8_t tx_data[8] = {/* command bytes */};
uint8_t rx_data[8];

// Manually handle CS (chip select)
DEV_GPIO_WriteValue(RADIO_CS, 0);  // CS active (low)

// Send command
DEV_SPI_Write_Bytes(tx_data, 8);

// Read response
DEV_SPI_Read_Bytes(rx_data, 8);

DEV_GPIO_WriteValue(RADIO_CS, 1);  // CS inactive (high)

// Process rx_data...
```

#### Register Read
```c
uint8_t read_register(uint8_t reg_address) {
    DEV_GPIO_WriteValue(RADIO_CS, 0);
    
    // Send read command
    uint8_t cmd = 0x80 | reg_address;  // Read opcode + address
    DEV_SPI_Write_Bytes(&cmd, 1);
    
    // Read register value
    uint8_t value;
    DEV_SPI_Read_Bytes(&value, 1);
    
    DEV_GPIO_WriteValue(RADIO_CS, 1);
    
    return value;
}
```

#### Register Write
```c
void write_register(uint8_t reg_address, uint8_t value) {
    DEV_GPIO_WriteValue(RADIO_CS, 0);
    
    // Send write command and data
    uint8_t cmd[2] = {reg_address, value};
    DEV_SPI_Write_Bytes(cmd, 2);
    
    DEV_GPIO_WriteValue(RADIO_CS, 1);
}
```

#### Buffer Transfer with Status
```c
// Send data buffer and get status byte
void send_data_buffer(const uint8_t* data, uint8_t length) {
    DEV_GPIO_WriteValue(RADIO_CS, 0);
    
    // Write opcode
    uint8_t opcode = 0x7F;  // Example: FIFO write command
    DEV_SPI_Write_Bytes(&opcode, 1);
    
    // Write data
    DEV_SPI_Write_Bytes(data, length);
    
    // Read status byte
    uint8_t status;
    DEV_SPI_Read_Bytes(&status, 1);
    
    DEV_GPIO_WriteValue(RADIO_CS, 1);
    
    printf("Transmission status: 0x%02X\n", status);
}
```

---

## SPI Timing

### Clock Frequency
- **Current:** 1 MHz (configurable)
- **Maximum:** Limited by LR1121 spec (typically 10-20 MHz)
- **Minimum:** No minimum specified

**Byte Transfer Time at 1 MHz:**
- 1 byte: ~8 µs
- 32 bytes: ~260 µs

### Chip Select Timing
- CS must be inactive (high) between transfers
- Minimum inactive time: Device dependent (typically 100 ns)
- Can bring CS low immediately for back-to-back transfers

### Typical Transfer Sequence
```
                   ┌──────────────────────────┐
CS                 │                          │
        ───────────┘                          └──────
        
        ↑           ↑                          ↑
        |           |                          |
        CS active   Transfer in progress       CS inactive
        (0)         (8 bits/µs)                (1)
        
        Timing: CS low → data transfer → CS high
```

---

## Current Hardware State

### SPI Interface (LR1121 Radio)
| Signal | GPIO | Direction | Function |
|--------|------|-----------|----------|
| SCLK | 10 | Output | SPI Clock |
| MOSI | 11 | Output | Data to Radio |
| MISO | 12 | Input | Data from Radio |
| CS | 13 | Output | Chip Select (manual) |

### Radio Control Signals
| Signal | GPIO | Direction | Function |
|--------|------|-----------|----------|
| RESET | 8 | Output | Radio Reset (active low) |
| BUSY | 9 | Input | Radio Busy Flag (active high) |
| IRQ | 14 | Input | Interrupt Request (falling edge) |

### Status Indicator
| Signal | GPIO | Direction | Function |
|--------|------|-----------|----------|
| LED | 25 | Output | Activity Indicator |

---

## Hardware Abstraction Guidelines

### Do's ✓
- Use `DEV_GPIO_WriteValue()` for output control
- Use `DEV_GPIO_ReadValue()` for input sensing
- Use `DEV_GPIO_Mode()` to initialize pins
- Keep SPI transactions short
- Assert CS during multi-byte transfers

### Don'ts ✗
- Don't call SPI functions from interrupt handlers
- Don't modify GPIO configuration after initialization
- Don't assume CS is managed automatically (must control manually)
- Don't perform blocking operations in interrupt handlers
- Don't share SPI port between multiple devices without arbitration

---

## Performance Notes

### GPIO Operations
| Operation | Time |
|-----------|------|
| `DEV_GPIO_WriteValue()` | ~100 ns |
| `DEV_GPIO_ReadValue()` | ~100 ns |
| Pin state change propagation | ~10 ns |

### SPI Operations
| Operation | Time |
|-----------|------|
| SPI word (8 bits) at 1 MHz | ~8 µs |
| SPI 32-byte buffer | ~260 µs |
| CS setup/hold | ~50 ns |

### Total Transaction Time
```
CS low ────────────────────────────
          SPI transfer (~260 µs for 32 bytes)
                            CS high
        ├─ Software overhead (~1 µs) ─┤
        ├─ Actual transfer time ──────┤
Total transaction: ~5-10 µs overhead + transfer time
```

---

## Troubleshooting

### SPI Communication Failures
1. Verify GPIO 10-13 are not in use elsewhere
2. Check SPI clock frequency is compatible with device
3. Ensure CS is toggled correctly (high between transfers)
4. Verify MISO pull-up (should be present on board)
5. Check for signal integrity at 1 MHz

### GPIO Input Not Reading
1. Verify pull-up is enabled
2. Check pin is in input mode
3. Inspect signal with oscilloscope
4. Verify external circuit is driving pin correctly

### GPIO Output Not Driving
1. Check pin is configured as output
2. Verify output can source sufficient current
3. Look for DC load limiting resistors
4. Check pin not in use by peripheral (SPI, UART, etc.)

---

## API Reference Checklist

### GPIO Module
- [ ] Call `DEV_GPIO_Mode()` for each GPIO pin before use
- [ ] Use `DEV_GPIO_WriteValue()` to drive outputs
- [ ] Use `DEV_GPIO_ReadValue()` to read inputs
- [ ] Register interrupt handlers with `DEV_GPIO_INT()`
- [ ] Keep interrupt handlers brief
- [ ] Verify pin modes don't conflict with peripherals

### SPI Module
- [ ] Call `DEV_SPI_Init()` once during startup
- [ ] Control CS manually with GPIO
- [ ] Assert CS before transfers, release after
- [ ] Use `DEV_SPI_Write_Bytes()` for write-only operations
- [ ] Use `DEV_SPI_Read_Bytes()` for read-only operations
- [ ] For full-duplex, do separate write then read
- [ ] Verify SPI clock speed with device datasheet

---

## Related Modules
- **LR1121 TX Module** - Uses SPI and GPIO for radio control
- **MCP2515 CAN Driver** - Uses SPI for CAN communication
- **FS26-DAQ Main** - Initializes GPIO/SPI during startup
