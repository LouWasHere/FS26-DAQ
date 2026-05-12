# GPS Module Integration Guide

## Overview

The GPS module provides NMEA-based positioning data from a UART-connected GPS receiver. It automatically handles baud rate detection, NMEA sentence parsing, data validation, and provides thread-safe access across cores via spin locks.

**Hardware Interface:** UART0 (GPIO 0/1)  
**Baud Rate:** 57600 bps  
**Update Rate:** 5 Hz (200ms between updates)  
**Protocol:** NMEA 0183

---

## Module API

### Include File
```c
#include "gps.h"
```

### Data Structure

```c
typedef struct {
    // Raw position and altitude
    bool fix_valid;              // True if GPS has valid fix
    float raw_latitude;          // Degrees (decimal)
    float raw_longitude;         // Degrees (decimal)
    float altitude;              // Meters
    
    // Motion
    float speed_kph;             // km/h
    float course;                // Degrees (true north)
    
    // Signal quality
    float hdop;                  // Horizontal Dilution of Precision (lower = better)
    int satellites;              // Number of satellites in solution
    
    // Filtered/display values
    float display_latitude;      // Latitude after filtering
    float display_longitude;     // Longitude after filtering
    bool is_moving;              // True if speed exceeds MIN_SPEED_THRESHOLD
} gps_data_t;
```

### Public Functions

#### `void gps_init(void)`
Initialize the GPS module with UART0 at 57600 baud and send configuration commands.

**Called from:** Core 0 startup (before main loop)

```c
void main() {
    stdio_init_all();
    
    // Initialize GPS module
    gps_init();  // UART0 starts listening for NMEA sentences
    
    // Main loop...
}
```

**Side Effects:**
- Configures UART0 (GPIO 0 TX, GPIO 1 RX)
- Sends MTK3339 commands to set 5 Hz update rate
- Sends configuration to output only GGA and RMC sentences
- Initializes spin lock for thread-safe data access

---

#### `void gps_process(void)`
Process incoming UART data and parse NMEA sentences. Non-blocking, should be called frequently (recommended every 100 µs or more).

**Called from:** Core 0 main loop

```c
while (true) {
    gps_process();      // Check UART buffer and parse complete sentences
    
    // Do other work...
    sleep_us(100);      // Don't block completely
}
```

**Operation:**
1. Reads available bytes from UART RX buffer
2. Accumulates characters until newline encountered
3. Validates NMEA checksum
4. Parses sentence type (GPGGA or GPRMC)
5. Updates `gps_data` structure with spin lock protection
6. Resets buffer for next sentence

**Performance:** ~50-100 µs per sentence (when sentence is complete)

---

#### `bool gps_is_readable(void)`
Check if UART RX has available data without blocking.

```c
if (gps_is_readable()) {
    gps_process();
}
```

---

#### `const gps_data_t* gps_get_data(void)`
Get pointer to current GPS data structure (read-only).

⚠️ **Warning:** NOT thread-safe. Use only from Core 0 or add external locking.

```c
const gps_data_t* gps = gps_get_data();
printf("Lat: %.6f, Lon: %.6f\n", gps->raw_latitude, gps->raw_longitude);
```

---

#### `void gps_get_data_safe(gps_data_t* out)`
Get thread-safe copy of GPS data. Uses spin lock internally.

✅ **Recommended for multi-core access.** Safe to call from any core.

```c
gps_data_t gps;
gps_get_data_safe(&gps);  // Atomic copy via spin lock

printf("Fix: %s, Sats: %d\n", 
       gps.fix_valid ? "VALID" : "INVALID", 
       gps.satellites);
```

**Parameters:**
- `out` - Pointer to `gps_data_t` struct to receive data copy

**Performance:** ~5-10 µs (includes spin lock acquire/release)

---

## Coordinate System

### Latitude/Longitude Format
- **Type:** Decimal degrees (WGS84)
- **Range:** Latitude ±90°, Longitude ±180°
- **Precision:** Single precision float (~7 significant digits)
- **Accuracy:** Depends on GPS accuracy class (typically ±5-10 meters)

**Examples:**
```
Boston, MA:              42.3601° N, -71.0589° W
San Francisco, CA:       37.7749° N, -122.4194° W
Sydney, Australia:       -33.8688° S, 151.2093° E
```

### Positive/Negative Conventions
- **Latitude:** Positive = North, Negative = South
- **Longitude:** Positive = East, Negative = West

### Altitude
- **Type:** Meters above Mean Sea Level (MSL)
- **Accuracy:** Typically ±15-20 meters

---

## NMEA Sentence Support

### Supported Sentences

#### GPGGA (Global Positioning System Fix Data)
Provides:
- Latitude/Longitude
- Number of satellites
- HDOP (Horizontal Dilution of Precision)
- Altitude

**Parse Interval:** Every sentence received (nominally 200 ms at 5 Hz)

#### GPRMC (Recommended Minimum Navigation Information)
Provides:
- Speed over ground (in knots, converted to km/h)
- Course over ground (true heading)
- Fix validity status

**Parse Interval:** Every sentence received (nominally 200 ms at 5 Hz)

---

## Signal Quality Metrics

### HDOP (Horizontal Dilution of Precision)
Indicates the quality of the GPS solution geometry.

| HDOP Value | Rating | Quality |
|-----------|--------|---------|
| 1-2 | Excellent | Excellent geometry |
| 2-5 | Good | Good geometry |
| 5-10 | Moderate | Moderate geometry |
| 10-20 | Fair | Fair geometry |
| 20+ | Poor | Poor geometry |

**Filter:** `MAX_HDOP_THRESHOLD = 3.0f` (rejects HDOP > 3.0)

### Satellite Count
- **Minimum for fix:** 4 satellites
- **Typical for good fix:** 8-12 satellites
- **Excellent:** 15+ satellites

---

## Data Filtering

### Speed Filter
**Threshold:** `MIN_SPEED_THRESHOLD = 8.0f` km/h

- Sets `is_moving = true` when speed exceeds 8 km/h
- Helps distinguish GPS noise from actual movement
- Typical GPS noise floor is ~7 km/h

**Use Case:**
```c
gps_data_t gps;
gps_get_data_safe(&gps);

if (gps.is_moving && gps.fix_valid) {
    // Vehicle is definitely moving and has valid fix
    process_velocity_data(gps.speed_kph, gps.course);
}
```

### Accuracy Filter
**Threshold:** `MAX_HDOP_THRESHOLD = 3.0f`

- Rejects fixes with HDOP > 3.0
- Only updates position data when HDOP is good
- Prevents jumping to inaccurate positions

---

## Time Synchronization

GPS data updates at **5 Hz** (200 ms intervals). Data in the `gps_data_t` structure represents a snapshot of the GPS receiver's state at the moment the NMEA sentence was parsed.

**Note:** The module does NOT provide absolute time information. For timestamped telemetry, you must:
1. Call `get_absolute_time()` from pico/stdlib.h when reading GPS data
2. Or use GPS time from extended NMEA sentences (not currently parsed)

---

## Common Usage Patterns

### Basic GPS Polling (Core 0)
```c
int main() {
    stdio_init_all();
    gps_init();
    
    while (true) {
        gps_process();  // Parse available NMEA sentences
        
        // Read current GPS state
        const gps_data_t* gps = gps_get_data();
        
        if (gps->fix_valid) {
            printf("Lat: %.6f, Lon: %.6f, Speed: %.1f km/h\n",
                   gps->raw_latitude, gps->raw_longitude, gps->speed_kph);
        }
        
        sleep_us(100);
    }
}
```

### Thread-Safe Multi-Core Access
```c
// Core 0 - GPS acquisition
void core0_main() {
    gps_init();
    while (true) {
        gps_process();
        sleep_us(100);
    }
}

// Core 1 - LoRa Telemetry Broadcasting
void core1_main() {
    lora_tx_init();
    
    while (true) {
        gps_data_t gps;
        gps_get_data_safe(&gps);  // Thread-safe copy
        
        // Build and send telemetry with GPS data
        my_packet_t pkt;
        pkt.latitude = gps.raw_latitude;
        pkt.longitude = gps.raw_longitude;
        lora_send((uint8_t*)&pkt, sizeof(pkt));
        
        sleep_ms(1000);
    }
}
```

### Conditional Logging (Only Log Valid Fixes)
```c
gps_data_t gps;
gps_get_data_safe(&gps);

if (gps.fix_valid && gps.satellites >= 4 && gps.hdop < 3.0f) {
    safe_printf("[GPS] Lat=%.6f Lon=%.6f Alt=%.1f Sats=%d HDOP=%.2f\n",
                gps.raw_latitude, gps.raw_longitude, gps.altitude,
                gps.satellites, gps.hdop);
} else {
    safe_printf("[GPS] No fix: Sats=%d HDOP=%.2f\n",
                gps.satellites, gps.hdop);
}
```

### Distance Calculation (Haversine Formula)
```c
#include <math.h>

float gps_haversine_distance(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371000.0f;  // Earth radius in meters
    
    float dlat = (lat2 - lat1) * M_PI / 180.0f;
    float dlon = (lon2 - lon1) * M_PI / 180.0f;
    float a = sinf(dlat/2) * sinf(dlat/2) +
              cosf(lat1 * M_PI / 180.0f) * cosf(lat2 * M_PI / 180.0f) *
              sinf(dlon/2) * sinf(dlon/2);
    float c = 2.0f * asinf(sqrtf(a));
    
    return R * c;  // Distance in meters
}
```

### Waypoint Tracking
```c
typedef struct {
    float latitude;
    float longitude;
    float threshold_distance;  // meters
} waypoint_t;

bool gps_waypoint_reached(const gps_data_t* gps, const waypoint_t* wp) {
    if (!gps->fix_valid) return false;
    
    float distance = gps_haversine_distance(
        gps->raw_latitude, gps->raw_longitude,
        wp->latitude, wp->longitude
    );
    
    return distance < wp->threshold_distance;
}
```

---

## Configuration

### Modifying Update Rate
Edit `gps.h`:
```c
#define GPS_CMD_RATE        "$PMTK220,200*2C\r\n"    // 200ms = 5Hz
```

Common rates (MTK3339 commands):
- 1000 ms (1 Hz): `$PMTK220,1000*1F\r\n`
- 500 ms (2 Hz): `$PMTK220,500*2B\r\n`
- 200 ms (5 Hz): `$PMTK220,200*2C\r\n`
- 100 ms (10 Hz): `$PMTK220,100*2F\r\n`

### Modifying Output Sentences
Edit `gps.h`:
```c
#define GPS_CMD_SET_OUTPUT  "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
```

Format: `$PMTK314,<GLL>,<RMC>,<VTG>,<GGA>,<GSA>,<GSV>,<GRS>,<GST>,<Null>,<Null>,<Null>,<Null>,<Null>,<Null>,<Null>,<Null>,<Null>,<Null>,<Null>,<Checksum>*<HEX>\r\n`

- 0 = disabled, 1 = every sentence, 5 = every 5th sentence

Current: GLL(0), RMC(1), VTG(0), GGA(1) - only RMC and GGA

### Modifying Filtering Thresholds
Edit `gps.h`:
```c
#define MAX_HDOP_THRESHOLD  3.0f   // Reject HDOP > 3.0
#define MIN_SPEED_THRESHOLD 8.0f   // km/h for is_moving flag
```

---

## Troubleshooting

### GPS Not Getting Fix
1. Check antenna connection (should be outside or near window)
2. Verify 5+ satellites detected
3. Check HDOP value (should be < 5)
4. Wait 1-2 minutes for initial acquisition
5. Monitor UART data: `gps_is_readable()` should return true frequently

### Erratic Position Data
1. Check HDOP threshold (should be < 3.0)
2. Verify antenna quality
3. Look for multipath sources (reflective surfaces)
4. Increase update interval (reduce from 5 Hz to 2 Hz or 1 Hz)

### UART Communication Issues
1. Verify GPIO 0 (TX) and GPIO 1 (RX) are correctly wired
2. Check baud rate 57600 matches receiver
3. Ensure pull-up resistors on RX line (internal pull-up enabled)
4. Check UART buffer size (256 bytes) is sufficient

### Spin Lock Deadlock
- Ensure `gps_get_data_safe()` is never called from within interrupt handlers
- Always use `gps_get_data()` read-only access from Core 0
- Use `gps_get_data_safe()` from Core 1

---

## Performance Notes

- **UART Processing:** ~50-100 µs per complete NMEA sentence
- **Data Copy (safe):** ~5-10 µs including spin lock
- **Data Copy (unsafe):** <1 µs
- **Memory:** ~180 bytes for gps_data_t struct
- **UART Buffer:** 256 bytes for incoming sentences
- **Minimum Call Frequency:** Process every ~1 ms to avoid buffer overflow at 57600 baud

---

## API Reference Checklist

- [ ] Call `gps_init()` once at startup from Core 0
- [ ] Call `gps_process()` frequently in main loop (every 100 µs)
- [ ] Use `gps_get_data_safe()` for multi-core access
- [ ] Use `gps_get_data()` for Core 0 only access (faster)
- [ ] Check `gps->fix_valid` before trusting position data
- [ ] Monitor `gps->satellites` and `gps->hdop` for signal quality
- [ ] Use `gps->is_moving` flag to filter GPS noise
- [ ] Apply coordinate validation in critical sections
- [ ] Handle case where GPS returns (0, 0) for invalid fix

---

## Related Modules
- **Safe Print** (`safe_print.h`) - Thread-safe printf macro
- **FS26-DAQ Main** - Uses GPS data in combined telemetry packet
- **CAN Handler** - Synchronizes GPS data with CAN sensor data
