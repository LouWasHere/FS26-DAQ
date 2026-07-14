# LoRa TX Module (LR1121) Integration Guide

## Overview

The LoRa TX module provides a simplified TX-only interface to the Semtech LR1121 LoRa radio transceiver. It handles SPI communication, radio initialization, modulation configuration, and packet transmission operations. 

**Hardware Interface:** SPI1 (GPIO 10-13)  
**Radio Chip:** LR1121 (Standalone chip, through-hole mounted. Not a Pico HAT.)  
**Frequency:** Natively 2.4 GHz (2400 MHz ISM Band)  
**Modulation:** LoRa  
**TX Power:** 13 dBm (~20 mW)  
**Base Maximum Payload:** 32 bytes (Supports multi-packet chunking for 68-byte telemetry)  
**Spreading Factor:** SF7 (good range/speed tradeoff)

---

## Module API

### Include File
```c
#include "lr1121_tx.h"
```

### Public Functions

#### `void lora_tx_init(void)`
Initialize the LR1121 radio for TX-only operation.

**Called from:** Core 1 startup

```c
void core1_main() {
    lora_tx_init();  // Configure radio, set frequency, modulation, power
    
    // Now ready to send packets
    while (true) {
        // Send telemetry...
    }
}
```

**Configuration Applied:**
- SPI clock speed: Configured via SPI module
- Radio frequency: 2400 MHz
- TX power: 13 dBm
- Spreading factor: SF7
- Bandwidth: 800 kHz
- Coding rate: 4/5
- Preamble: 8 symbols
- CRC: Disabled
- IQ inversion: Standard (not inverted)
- Syncword: 0x12 (private network)

**Side Effects:**
- Initializes SPI1
- Configures GPIO pins (CS, BUSY, IRQ, RESET)
- Resets and initializes LR1121 chip
- Loads calibration data into radio memory

**Performance:** ~1-2 seconds initialization time

---

#### `bool lora_send(const uint8_t* data, uint8_t length)`
Send data over LoRa. Blocking call that waits for TX to complete.

**Parameters:**
- `data` - Pointer to payload buffer
- `length` - Number of bytes to send (1-32 bytes)

**Returns:**
- `true` - TX completed successfully
- `false` - TX timeout or error

```c
uint8_t telemetry_packet[32];
// ... fill telemetry_packet with data ...

if (lora_send(telemetry_packet, 32)) {
    printf("TX successful\n");
} else {
    printf("TX failed or timed out\n");
}
```

**Blocking Behavior:**
- Waits for TX completion (typically 50-500 ms depending on SF and payload)
- Cannot interrupt or cancel TX once started
- Recommended to call from Core 1 to keep Core 0 responsive

**Payload Constraints:**
```c
#define PAYLOAD_LENGTH 32  // Maximum 32 bytes per transmission

// Valid calls:
lora_send(buffer, 1);       // Single byte
lora_send(buffer, 16);      // Typical telemetry packet
lora_send(buffer, 32);      // Maximum size

// Invalid:
lora_send(buffer, 33);      // ERROR - exceeds max
lora_send(buffer, 0);       // ERROR - no data
```

**Timing Example (SF7):**
- 16-byte payload: ~60 ms airtime
- 32-byte payload: ~100 ms airtime
- At 1 Hz broadcast rate: leaves 900+ ms for other tasks

---

#### `uint32_t lora_get_tx_count(void)`
Get the current TX packet counter.

**Returns:** Number of packets transmitted since `lora_tx_init()`

```c
while (true) {
    if (lora_send(packet, 32)) {
        uint32_t count = lora_get_tx_count();
        printf("Packet #%lu transmitted\n", count);
    }
    
    sleep_ms(1000);  // 1 Hz broadcast
}
```

**Use Cases:**
- Track transmission success rate
- Include packet number in telemetry for receiver validation
- Detect if TX module has hung

---

## LoRa Configuration Parameters

### RF Parameters

#### Frequency
```c
#define RF_FREQ_IN_HZ 2400000000UL  // 2400 MHz (2.4 GHz ISM Band)
```

**Supported Frequencies:**
- ISM Band: 2400-2500 MHz (typical for LoRa)
- Must match receiver frequency exactly
- Verify regulatory compliance for your region

#### TX Power
```c
#define TX_OUTPUT_POWER_DBM 13  // Range: -9 to 22 dBm
```

| dBm | Approx mW | Distance | Use Case |
|-----|-----------|----------|----------|
| -9  | 0.8 mW    | <500m    | Very low power |
| 0   | 1 mW      | 1-2 km   | Standard |
| 13  | 20 mW     | 5-10 km  | **Current** |
| 22  | 158 mW    | 15+ km   | Maximum range |

**Higher power = longer range but higher current draw and heat**

#### Modulation (LoRa)
```c
#define PACKET_TYPE LR11XX_RADIO_PKT_TYPE_LORA
```

Alternative: `LR11XX_RADIO_PKT_TYPE_GFSK` (FSK modulation, not used)

### Modulation Parameters (LoRa)

#### Spreading Factor
```c
#define LORA_SPREADING_FACTOR LR11XX_RADIO_LORA_SF7
```

| SF | Airtime | Range | Speed | Use Case |
|----|---------|-------|-------|----------|
| SF7 | ~60 ms | Medium | Fast | **Current** - balanced |
| SF8 | ~120 ms | Medium-Long | Medium | Better range |
| SF9 | ~240 ms | Long | Slower | Weak signal |
| SF10 | ~480 ms | Long | Very slow | Extreme range |
| SF11 | ~1 sec | Very long | Very slow | Marginal signals |
| SF12 | ~2 sec | Very long | Slowest | Last resort |

**Current setting (SF7):** 60 ms airtime for 16-byte packet, 100 ms for 32-byte

#### Bandwidth
```c
#define LORA_BANDWIDTH LR11XX_RADIO_LORA_BW_800  // 800 kHz
```

| BW | Data Rate | Sensitivity | Adjacent Band |
|----|-----------|-------------|---------------|
| 50 kHz | Low | Best | Tight |
| 100 kHz | Medium | Good | Tight |
| 200 kHz | Medium-High | OK | Moderate |
| 500 kHz | High | Fair | Wide |
| 800 kHz | High | Fair | **Current** |
| 1600 kHz | Very high | Poor | Very wide |

**Higher BW = faster data, worse sensitivity**

#### Coding Rate
```c
#define LORA_CODING_RATE LR11XX_RADIO_LORA_CR_4_5
```

| CR | Overhead | Range Penalty | Error Recovery |
|----|----------|---------------|----------------|
| 4/5 | 0% | None | **Current** - baseline |
| 4/6 | 20% | +0.5 dB | Better error correction |
| 4/7 | 40% | +1.5 dB | Better error correction |
| 4/8 | 60% | +2.0 dB | Best error correction |

**Higher CR = more resilient to noise, slower transmission**

### Packet Parameters

#### Preamble
```c
#define LORA_PREAMBLE_LENGTH 8  // symbols
```

Standard: 6-8 symbols. More symbols = better range, slower TX start.

#### Packet Length Mode
```c
#define LORA_PKT_LEN_MODE LR11XX_RADIO_LORA_PKT_EXPLICIT
```

- **Explicit:** Length field in packet (current setting)
- **Implicit:** Length fixed, must be known by receiver

#### IQ Inversion
```c
#define LORA_IQ LR11XX_RADIO_LORA_IQ_STANDARD
```

- **Standard:** Normal IQ
- **Inverted:** For special link budgets
- Must match receiver setting

#### CRC
```c
#define LORA_CRC LR11XX_RADIO_LORA_CRC_OFF
```

- **Off:** No CRC (current setting) - faster
- **On:** CRC validation - error detection

#### Syncword
```c
#define LORA_SYNCWORD 0x12
```

- **0x12:** Private network (current) - avoids LoRaWAN interference
- **0x34:** Public LoRaWAN network
- Must match receiver setting

---

## Packet Structure

### Binary Packet Format

```
Byte 0          | Bytes 1-N
════════════════╪═══════════════════════
Preamble Sync   | LoRa Header
(8 symbols)     | Length (optional, explicit mode)
                | Payload Data
```

### Example Telemetry Packet (32 bytes)

```c
typedef struct __attribute__((packed)) {
    uint32_t magic;              // 4 bytes - 0x46533236 ("FS26")
    float latitude;              // 4 bytes
    float longitude;             // 4 bytes
    float speed_kph;             // 4 bytes
    float altitude;              // 4 bytes
    uint16_t rpm;                // 2 bytes
    uint16_t tx_count;           // 2 bytes
} telemetry_packet_t;            // 24 bytes total

// Send this packet:
telemetry_packet_t pkt;
pkt.magic = 0x46533236;
pkt.latitude = 42.3601f;
// ... populate fields ...

lora_send((uint8_t*)&pkt, sizeof(pkt));  // 24 bytes
```

### Variable-Length Packets

```c
// Send only essential data
uint8_t minimal_packet[8];
minimal_packet[0] = 0x46;  // 'F'
minimal_packet[1] = 0x53;  // 'S'
// ... fill with data ...

lora_send(minimal_packet, 8);  // Much faster TX

// Send complete data
uint8_t full_packet[32];
// ... fill all 32 bytes ...

lora_send(full_packet, 32);  // Slower but more data
```

---

## Air Time Calculations

Formula: `Airtime = (2^SF / BW) × (PL + 4.25) × CR`

Where:
- SF = Spreading Factor (7-12)
- BW = Bandwidth in Hz (50k to 1.6M)
- PL = Payload length in bytes
- CR = Coding rate (4/5 = 1.25, 4/6 = 1.5, etc.)

### Typical Times (SF7, BW800, CR4/5)
```
Payload | Airtime
────────┼──────────
1 byte  | ~41 ms
8 bytes | ~50 ms
16 bytes| ~60 ms
24 bytes| ~80 ms
32 bytes| ~100 ms
```

**Total Transmission Time = Airtime (typical 100ms for 32-byte at SF7)**

---

## Common Usage Patterns

### Periodic Broadcast (1 Hz)
```c
void core1_main() {
    lora_tx_init();
    
    while (true) {
        combined_telemetry_packet_t packet;
        
        // Populate packet with GPS + CAN data
        build_telemetry(&packet);
        
        // TX takes ~100 ms, leaves 900 ms for other tasks
        if (lora_send((uint8_t*)&packet, sizeof(packet))) {
            uint32_t tx_num = lora_get_tx_count();
            safe_printf("[LoRa] TX #%lu successful\n", tx_num);
        } else {
            safe_printf("[LoRa] TX #%lu FAILED\n", lora_get_tx_count());
        }
        
        sleep_ms(1000);  // 1 Hz rate
    }
}
```

### Event-Based Transmission
```c
#define BUFFER_SIZE 20

typedef struct {
    uint32_t timestamp;
    uint8_t event_type;
    int16_t param1;
    int16_t param2;
} event_packet_t;

event_packet_t events[BUFFER_SIZE];
int event_count = 0;

// Accumulate events...

if (event_count >= BUFFER_SIZE / 2) {
    // Send accumulated events
    for (int i = 0; i < event_count; i++) {
        lora_send((uint8_t*)&events[i], sizeof(event_packet_t));
    }
    event_count = 0;
}
```

### Transmission with Retry
```c
bool lora_send_with_retry(const uint8_t* data, uint8_t length, int max_retries) {
    for (int attempt = 1; attempt <= max_retries; attempt++) {
        if (lora_send(data, length)) {
            return true;  // Success
        }
        
        safe_printf("[LoRa] TX attempt %d failed, retrying...\n", attempt);
        
        if (attempt < max_retries) {
            sleep_ms(100 + (attempt * 50));  // Exponential backoff
        }
    }
    
    return false;  // All retries failed
}

// Usage:
if (lora_send_with_retry(packet, 32, 3)) {
    safe_printf("[LoRa] TX successful after retry\n");
} else {
    safe_printf("[LoRa] TX failed after 3 attempts\n");
}
```

### Low-Power Selective Transmission
```c
// Only TX when critical data changes
uint8_t last_packet[32] = {0};
uint8_t current_packet[32];

while (true) {
    build_packet(current_packet);
    
    // Only TX if data significantly changed
    if (packet_changed(last_packet, current_packet)) {
        lora_send(current_packet, 32);
        memcpy(last_packet, current_packet, 32);
    }
    
    sleep_ms(100);
}
```

### Multi-Packet Transmission
```c
typedef struct {
    uint8_t packet_num;     // 0, 1, 2, ...
    uint8_t total_packets;  // 3, 3, 3, ...
    uint8_t data[30];
} chunked_packet_t;

uint8_t large_data[100];  // Too big for single 32-byte packet

void send_large_data(const uint8_t* data, uint16_t size) {
    int total_packets = (size + 29) / 30;  // Ceiling division
    
    for (int i = 0; i < total_packets; i++) {
        chunked_packet_t pkt;
        pkt.packet_num = i;
        pkt.total_packets = total_packets;
        
        int copy_size = (size - (i * 30) > 30) ? 30 : (size - (i * 30));
        memcpy(pkt.data, &data[i * 30], copy_size);
        
        lora_send((uint8_t*)&pkt, sizeof(pkt));
        sleep_ms(200);  // Spacing between packets
    }
}
```

---

## Performance & Power Considerations

### TX Timing
| Operation | Time | Notes |
|-----------|------|-------|
| `lora_tx_init()` | ~1-2 sec | One-time startup |
| `lora_send()` setup | ~1-5 ms | SPI config, radio setup |
| `lora_send()` TX | ~50-100 ms | Actual airtime (SF7) |
| `lora_send()` total | ~60-110 ms | End-to-end blocking call |
| `lora_get_tx_count()` | <1 µs | Simple counter read |

### Current Draw
| State | Current | Notes |
|-------|---------|-------|
| Idle (initialized) | ~1 mA | Radio in standby |
| TX (13 dBm) | ~200-300 mA | Peak during transmission |
| Average (1 Hz, 100 ms TX) | ~3-5 mA | With other systems idle |

### Range Estimate
| Condition | Range | Notes |
|-----------|-------|-------|
| Urban (LoS) | 5-10 km | Line of sight |
| Urban (NLoS) | 1-2 km | Buildings blocking signal |
| Rural (LoS) | 10-20 km | Open terrain |
| Over water | 20+ km | Excellent propagation |

---

## Troubleshooting

### TX Always Fails
1. Check SPI connection (GPIO 10-13)
2. Verify LR1121 chip power supply
3. Check `lora_get_tx_count()` - is it incrementing before failure?
4. Verify antenna connection
5. Check RF frequency is legal in your region

### TX Works but No Reception
1. Verify receiver frequency matches exactly (2400 MHz)
2. Check syncword matches (0x12 for current config)
3. Verify spreading factor and bandwidth match
4. Check antenna orientation on both sides
5. Test with LoRa analyzer to see transmitted signal

### Intermittent TX Failures
1. Check power supply stability (may sag during TX)
2. Verify SPI clock speed is within spec
3. Check for RF interference on 2.4 GHz band
4. Increase timeout value in `lora_send()`
5. Try lower TX power to reduce EMI

### Antenna Issues
- Check antenna impedance match (typically 50Ω)
- Verify antenna not damaged or bent
- Test antenna with network analyzer if available
- Try external antenna for better performance
- Keep antenna away from metal objects

---

## Advanced Configuration

### Custom Modulation Parameters
To modify SF, BW, or other parameters, edit `lr1121_config.h`:

```c
// Change to SF8 for better range (slower)
#define LORA_SPREADING_FACTOR LR11XX_RADIO_LORA_SF8

// Change to 500 kHz bandwidth (faster)
#define LORA_BANDWIDTH LR11XX_RADIO_LORA_BW_500

// Change to higher TX power
#define TX_OUTPUT_POWER_DBM 22
```

After modifying `lr1121_config.h`, rebuild and reflash the project.

### Custom Payload Length
Maximum is 32 bytes. To use larger payloads:

1. Modify `lr1121_config.h`:
   ```c
   #define PAYLOAD_LENGTH 64  // If supported by radio
   ```

2. Check LR1121 datasheet for actual maximum supported

3. Note: Larger payloads = slower TX, higher power consumption

---

## API Reference Checklist

- [ ] Call `lora_tx_init()` once at Core 1 startup
- [ ] Call `lora_send()` with valid data buffer and length (1-32 bytes)
- [ ] Handle return value (true = success, false = timeout)
- [ ] Use `lora_get_tx_count()` for packet numbering
- [ ] Include packet counter in telemetry for validation
- [ ] Use spin locks if sharing TX module between cores
- [ ] Verify antenna is properly connected
- [ ] Ensure receiver configuration matches (freq, SF, BW, syncword)
- [ ] Monitor TX success rate and adjust parameters if needed
- [ ] Be aware `lora_send()` is blocking - minimize other delays

---

## Related Modules
- **SPI Module** (`src/spi/spi.h`) - Low-level SPI communication
- **GPIO Module** (`src/gpio/gpio.h`) - Pin control and interrupts
- **LoRa Config** (`lr1121_config.h`) - RF parameters
- **FS26-DAQ Main** - Uses LoRa TX for telemetry broadcast
