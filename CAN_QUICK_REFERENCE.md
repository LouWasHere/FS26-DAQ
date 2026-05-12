# CAN Integration Quick Reference Guide

## Module Overview

### **ft550_decoder** - FT550 Protocol Support
Parse and decode CAN frames from FT550 ECU with automatic multiplier application.

**Include in your code:**
```c
#include "ft550_decoder.h"
```

**Basic Usage:**
```c
// Initialize sensor data once
ft550_sensor_data_t sensor_data;
ft550_init_sensor_data(&sensor_data);

// When CAN frame arrives:
uint8_t can_frame[8];
uint32_t frame_id = 0x14080600;  // TPS, MAP, Air Temp, Engine Temp
if (ft550_decode_frame(frame_id, can_frame, &sensor_data)) {
    printf("TPS: %.1f%%\n", sensor_data.tps);
    printf("Engine Temp: %.1f°C\n", sensor_data.engine_temp);
}
```

**All FT550 Frame IDs:**
```c
0x14080600  // TPS, MAP, Air Temp, Engine Temp
0x14080601  // Oil Pressure, Fuel Pressure, Water Pressure, Gear
0x14080602  // Exhaust O2, RPM, Oil Temp, Pit Limit
0x14080603  // Wheel Speeds (FR, FL, RR, RL)
0x14080604  // Traction Control, Heading
0x14080605  // Shock Sensors (FR, FL, RR, RL)
0x14080606  // G-Forces, Yaw Rates
0x14080607  // Lambda, Fuel Flow, Injection Time
0x14080608  // Transmission Temps, Fuel Consumption, Brake Pressure
```

**Key Data Fields:**
- Temperatures: `air_temp`, `engine_temp`, `oil_temp`, `trans_oil_temp`, `trans_temp`
- Pressures: `oil_pressure`, `fuel_pressure`, `water_pressure`, `brake_pressure`, `map`
- Engine: `rpm`, `tps`, `exhaust_o2`, `fuel_flow_total`, `fuel_consumption`
- Wheels: `wheel_speed_fr`, `wheel_speed_fl`, `wheel_speed_rr`, `wheel_speed_rl`
- Dynamics: `g_force_accel`, `g_force_lateral`, `yaw_rate_frontal`, `yaw_rate_lateral`, `heading`
- Other: `gear`, `pit_limit`, `lambda_correction`, `inj_time_bank_a`, `inj_time_bank_b`

---

### **can_handler** - CAN Bus Interface
Manage MCP2515 CAN controller and provide thread-safe sensor data access.

**Include in your code:**
```c
#include "can_handler.h"
```

**Initialization (call once on Core 1 startup):**
```c
can_init();  // Configures MCP2515 for 1 Mbps, extended 29-bit IDs
```

**Main Loop Integration:**
```c
while (true) {
    // Poll for incoming CAN frames (non-blocking)
    can_process_frame();
    
    // ... rest of your code ...
}
```

**Accessing Sensor Data (thread-safe):**
```c
ft550_sensor_data_t sensor_data;
can_get_sensor_data_safe(&sensor_data);  // Acquires spin lock internally

// Now safe to read:
printf("RPM: %u\n", sensor_data.rpm);
printf("Brake Pressure: %.3f Bar\n", sensor_data.brake_pressure);
```

**Monitoring:**
```c
uint32_t frame_count = can_get_frame_count();
printf("CAN frames processed: %lu\n", frame_count);
```

---

## Integration Pattern (Core 1)

```c
#include "can_handler.h"
#include "ft550_decoder.h"
#include "lr1121_tx.h"

void core1_main() {
    // Initialize LoRa and CAN
    lora_tx_init();
    can_init();
    
    while (true) {
        // Step 1: Poll for new CAN frames
        can_process_frame();
        
        // Step 2: Get latest sensor data (thread-safe)
        ft550_sensor_data_t can_data;
        can_get_sensor_data_safe(&can_data);
        
        // Step 3: Build telemetry packet with CAN data
        MyTelemetryPacket_t packet;
        packet.rpm = can_data.rpm;
        packet.engine_temp = can_data.engine_temp;
        packet.brake_pressure = can_data.brake_pressure;
        // ... populate more fields ...
        
        // Step 4: Send via LoRa
        lora_send((uint8_t*)&packet, sizeof(packet));
        
        sleep_ms(10);  // Or appropriate delay for your application
    }
}
```

---

## Data Type Reference

### Temperatures (multiply raw by 0.1)
| Sensor | Field | Units |
|--------|-------|-------|
| Air Intake | `air_temp` | °C |
| Engine Coolant | `engine_temp` | °C |
| Engine Oil | `oil_temp` | °C |
| Transmission Oil | `trans_oil_temp` | °C |
| Transmission Fluid | `trans_temp` | °C |

**Conversion:** Raw INT16 × 0.1 = °C
- Example: Raw value 850 = 85.0°C

### Pressures (multiply raw by 0.001)
| Sensor | Field | Units |
|--------|-------|-------|
| Manifold Absolute Pressure | `map` | Bar |
| Engine Oil | `oil_pressure` | Bar |
| Fuel Supply | `fuel_pressure` | Bar |
| Coolant System | `water_pressure` | Bar |
| Brake Hydraulic | `brake_pressure` | Bar |

**Conversion:** Raw INT16 × 0.001 = Bar
- Example: Raw value 2500 = 2.5 Bar (≈ 250 kPa)

### Engine Parameters
| Parameter | Field | Multiplier | Units |
|-----------|-------|-----------|-------|
| Throttle Position | `tps` | 0.1 | % |
| Engine Speed | `rpm` | 1 | RPM |
| Exhaust O₂ | `exhaust_o2` | 0.001 | λ (lambda) |
| Fuel Flow | `fuel_flow_total` | 0.01 | L/min |
| Fuel Consumed | `fuel_consumption` | 1 | L |

### Speed/Motion
| Parameter | Field | Multiplier | Units |
|-----------|-------|-----------|-------|
| Wheel Speed FR | `wheel_speed_fr` | 1 | km/h |
| Wheel Speed FL | `wheel_speed_fl` | 1 | km/h |
| Wheel Speed RR | `wheel_speed_rr` | 1 | km/h |
| Wheel Speed RL | `wheel_speed_rl` | 1 | km/h |
| G-force Accel | `g_force_accel` | 1 | g |
| G-force Lateral | `g_force_lateral` | 1 | g |
| Yaw Rate Frontal | `yaw_rate_frontal` | 1 | rad/s |
| Yaw Rate Lateral | `yaw_rate_lateral` | 1 | rad/s |
| Heading | `heading` | 1 | degrees |

### Suspension/Control
| Parameter | Field | Multiplier | Units |
|-----------|-------|-----------|-------|
| Shock Sensor FR | `shock_fr` | 0.001 | (normalized) |
| Shock Sensor FL | `shock_fl` | 0.001 | (normalized) |
| Shock Sensor RR | `shock_rr` | 0.001 | (normalized) |
| Shock Sensor RL | `shock_rl` | 0.001 | (normalized) |

### Other Parameters
| Parameter | Field | Multiplier | Units | Notes |
|-----------|-------|-----------|-------|-------|
| Gear Selection | `gear` | 1 | enum | 0=Neutral, 1=1st, etc. |
| Traction Ctrl Slip | `traction_ctrl_slip` | 1 | (raw) | |
| Traction Ctrl Retard | `traction_ctrl_retard` | 1 | (raw) | |
| Traction Ctrl Cut | `traction_ctrl_cut` | 1 | (raw) | |
| Lambda Correction | `lambda_correction` | 1 | (raw) | |
| Inj Time Bank A | `inj_time_bank_a` | 0.01 | ms | |
| Inj Time Bank B | `inj_time_bank_b` | 0.01 | ms | |
| Pit Limiter | `pit_limit` | 1 | (raw) | |

---

## Common Coding Patterns

### Extract Specific Measurements from Frame
```c
ft550_sensor_data_t sensor_data;
ft550_init_sensor_data(&sensor_data);

// Process frame 0x14080600 (TPS, MAP, Air Temp, Engine Temp)
uint8_t frame[8] = {/* data from CAN bus */};
ft550_decode_frame(0x14080600, frame, &sensor_data);

// Now access:
float tps_percent = sensor_data.tps;           // 0-100%
float map_bar = sensor_data.map;               // Absolute pressure
float engine_celsius = sensor_data.engine_temp; // °C
```

### Monitor Wheel Speeds & Calculate Average
```c
ft550_sensor_data_t can_data;
can_get_sensor_data_safe(&can_data);

uint16_t avg_speed = (can_data.wheel_speed_fr + 
                      can_data.wheel_speed_fl + 
                      can_data.wheel_speed_rr + 
                      can_data.wheel_speed_rl) / 4;

printf("Average wheel speed: %u km/h\n", avg_speed);
```

### Detect Pressure Anomalies
```c
ft550_sensor_data_t can_data;
can_get_sensor_data_safe(&can_data);

#define OIL_PRESSURE_MIN 1.5f  // Bar
#define OIL_PRESSURE_MAX 5.0f  // Bar

if (can_data.oil_pressure < OIL_PRESSURE_MIN) {
    safe_printf("WARNING: Low oil pressure: %.2f Bar\n", can_data.oil_pressure);
} else if (can_data.oil_pressure > OIL_PRESSURE_MAX) {
    safe_printf("WARNING: High oil pressure: %.2f Bar\n", can_data.oil_pressure);
}
```

### Track Frame Reception Rate
```c
static uint32_t last_frame_count = 0;

uint32_t current_count = can_get_frame_count();
uint32_t frames_received = current_count - last_frame_count;
last_frame_count = current_count;

printf("Frames received in last second: %lu\n", frames_received);
```

---

## Troubleshooting

### CAN Frames Not Being Received
1. Verify MCP2515 hardware is connected and powered
2. Check CAN bus termination (120Ω resistors at both ends)
3. Ensure ECU is transmitting at 1 Mbps on extended 29-bit IDs
4. Verify frame IDs match FT550 protocol (0x14080600-0x14080608)
5. Check `can_get_frame_count()` - should be incrementing

### Data Always Zero
1. Verify `can_process_frame()` is being called frequently
2. Check that `can_get_sensor_data_safe()` is called (not reading uninitialized data)
3. Ensure CAN frames contain valid sensor data (check with CAN analyzer)

### Compilation Errors
1. Include both `can_handler.h` and `ft550_decoder.h`
2. Ensure CMakeLists.txt has `add_subdirectory(./src/mcp2515)`
3. Verify `target_link_libraries` includes `mcp2515`

---

## Performance Notes

- **CAN Polling:** `can_process_frame()` is non-blocking, ~100-200 µs per call
- **Data Copy:** `can_get_sensor_data_safe()` ~10-20 µs (includes spin lock overhead)
- **Decoder:** Frame decoding ~5 µs per frame
- **Thread Safety:** Spin lock ensures atomic access without blocking interrupts
- **Memory:** `ft550_sensor_data_t` = 188 bytes on stack or heap

---

## API Checklist

- [ ] Include `can_handler.h` and `ft550_decoder.h`
- [ ] Call `can_init()` once at startup
- [ ] Call `can_process_frame()` in main loop
- [ ] Call `can_get_sensor_data_safe()` before accessing sensor data
- [ ] Use `can_get_frame_count()` for monitoring
- [ ] Apply correct multipliers when accessing raw values (handled automatically by decoder)
- [ ] Use `ft550_extract_int16_be()` / `ft550_extract_uint16_be()` if manual decoding needed
