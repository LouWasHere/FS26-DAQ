# FS26-DAQ Pin Allocation Analysis

## Current Pin Assignments

### Summary Table

| GPIO | Module | Function | Notes |
|------|--------|----------|-------|
| 0 | GPS | UART0 TX | ✅ Dedicated |
| 1 | GPS | UART0 RX | ✅ Dedicated |
| 2 | - | **AVAILABLE** | |
| 3 | - | **AVAILABLE** | |
| 4 | CAN | SPI0 MISO | ✅ Dedicated (SPI0) |
| 5 | CAN | CS (MCP2515) | ✅ Dedicated |
| 6 | CAN | SPI0 CLK | ✅ Dedicated (SPI0) |
| 7 | CAN | SPI0 MOSI | ✅ Dedicated (SPI0) |
| 8 | LoRa | RADIO_RESET | ⚠️ Marked as LCD_DC in DEV_Config.h |
| 9 | LoRa | RADIO_BUSY | ⚠️ Marked as LCD_CS in DEV_Config.h |
| 10 | LoRa | SPI1 CLK | ⚠️ Marked as LCD_CLK in DEV_Config.h |
| 11 | LoRa | SPI1 MOSI | ⚠️ Marked as LCD_MOSI in DEV_Config.h |
| 12 | LoRa | SPI1 MISO | ⚠️ Marked as LCD_RST in DEV_Config.h |
| 13 | LoRa | RADIO_CS | ⚠️ Marked as LCD_BL in DEV_Config.h |
| 14 | LoRa | RADIO_IRQ | ✅ Dedicated |
| 15-24 | - | **AVAILABLE** | |
| 25 | LoRa | RADIO_LED | ✅ Dedicated |
| 26-29 | - | **AVAILABLE** | |

---

## Conflict Analysis

### **ISSUE FOUND: DEV_Config.h Has Old LCD Pin Definitions** ⚠️

The `/src/mcp2515/Config/DEV_Config.h` file contains legacy LCD display pin definitions that **OVERLAP with the LR1121 LoRa radio pins**:

```c
// DEV_Config.h (OLD LCD CONFIG - NOT USED)
#define LCD_RST_PIN  12     ◄─── CONFLICT: LoRa MISO = GPIO 12
#define LCD_DC_PIN   8      ◄─── CONFLICT: LoRa RESET = GPIO 8
#define LCD_BL_PIN   13     ◄─── CONFLICT: LoRa CS = GPIO 13
#define LCD_CS_PIN   9      ◄─── CONFLICT: LoRa BUSY = GPIO 9
#define LCD_CLK_PIN  10     ◄─── CONFLICT: LoRa CLK = GPIO 10
#define LCD_MOSI_PIN 11     ◄─── CONFLICT: LoRa MOSI = GPIO 11
```

**Good News:** The `DEV_GPIO_Init()` function comments out these LCD pins, so they're **not actively configured at runtime**. The CAN module only initializes `MCP2515_CS_PIN (GPIO 5)`.

---

## Physical SPI Bus Separation ✅

The system actually uses **two different SPI buses**, which is excellent:

### SPI0 (MCP2515 CAN)
- **Clock:** GPIO 6 (SPI0_SCK)
- **MOSI:** GPIO 7 (SPI0_TX)
- **MISO:** GPIO 4 (SPI0_RX)
- **CS:** GPIO 5 (MCP2515_CS0_PIN)
- **Status:** ✅ Dedicated & isolated

### SPI1 (LR1121 LoRa)
- **Clock:** GPIO 10 (SPI1_SCK)
- **MOSI:** GPIO 11 (SPI1_TX)
- **MISO:** GPIO 12 (SPI1_RX)
- **CS:** GPIO 13 (RADIO_CS)
- **Status:** ✅ Dedicated & isolated

### Result: ✅ **NO RUNTIME CONFLICTS** (Different SPI buses)

---

## Detailed Pin Map

### UART0 - GPS Module
```
┌─────────────────────────────┐
│      GPS NMEA Receiver      │
├─────────────────────────────┤
│ GPIO 0 (Pico TX)  → RX pin  │
│ GPIO 1 (Pico RX) ← TX pin   │
│ Baud: 57600 bps            │
│ Protocol: NMEA 0183         │
└─────────────────────────────┘
```

### SPI0 - MCP2515 CAN Controller
```
┌──────────────────────────────────┐
│    MCP2515 CAN Bus Controller   │
├──────────────────────────────────┤
│ GPIO 4  (SPI0 MISO)   ← RX      │
│ GPIO 6  (SPI0 CLK)    ↔ CLK    │
│ GPIO 7  (SPI0 MOSI)   → TX      │
│ GPIO 5  (CS)          → CS      │
│ Baud: 1 Mbps                    │
│ Frame: CAN 2.0B Extended (29b)  │
└──────────────────────────────────┘
```

### SPI1 - LR1121 LoRa Radio
```
┌──────────────────────────────────┐
│     LR1121 LoRa Radio (2.4GHz)  │
├──────────────────────────────────┤
│ GPIO 10  (SPI1 CLK)    ↔ CLK    │
│ GPIO 11  (SPI1 MOSI)   → TX     │
│ GPIO 12  (SPI1 MISO)   ← RX     │
│ GPIO 13  (CS)          → CS     │
│ GPIO 8   (RESET)       → RST    │
│ GPIO 9   (BUSY)        ← BUSY   │
│ GPIO 14  (IRQ)         ← IRQ    │
│ GPIO 25  (LED)         → LED    │
│ TX Power: 13 dBm               │
│ Freq: 2.4 GHz (2400 MHz)        │
└──────────────────────────────────┘
```

---

## Recommendation

### Action: Clean up DEV_Config.h

The old LCD pin definitions should be removed or clearly marked as DEPRECATED to avoid future confusion:

**Option 1: Remove LCD definitions entirely** (Recommended)
```c
// Remove these lines from DEV_Config.h:
// #define LCD_RST_PIN  12
// #define LCD_DC_PIN   8
// #define LCD_BL_PIN   13
// #define LCD_CS_PIN   9
// #define LCD_CLK_PIN  10
// #define LCD_MOSI_PIN 11
// #define LCD_SCL_PIN  7
// #define LCD_SDA_PIN  6
```

**Option 2: Mark as deprecated**
```c
// DEPRECATED - LCD display not used in FS26-DAQ
// #define LCD_RST_PIN  12
// etc...
```

**Keep CAN-specific pins:**
```c
// MCP2515 CAN Controller (ACTIVELY USED)
#define SPI_CLK_PIN  6
#define SPI_MOSI_PIN 7
#define SPI_MISO_PIN 4
#define MCP2515_CS0_PIN  5
#define MCP2515_CS1_PIN  1
#define MCP2515_CS_PIN   MCP2515_CS0_PIN
```

---

## Available Pins for Future Expansion

| GPIO | Status | Use Case |
|------|--------|----------|
| 2-3 | Available | Future analog/digital I/O |
| 15-24 | Available | Future sensors, I2C, PWM |
| 26-29 | Available | ADC inputs (A0-A3) |

---

## Summary

✅ **No Runtime Conflicts** - Each module has its own dedicated pins and buses  
✅ **SPI Bus Separation** - GPS uses UART, CAN uses SPI0, Radio uses SPI1  
⚠️ **Legacy Code Issue** - DEV_Config.h contains old LCD definitions (not active)  
✅ **Recommended Action** - Remove LCD definitions from DEV_Config.h for clarity

**Current state is SAFE to operate**, but DEV_Config.h should be cleaned up to prevent future confusion.
