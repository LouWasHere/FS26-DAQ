#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"  // Add this
#include "gps.h"
#include "lr1121_ping_pong.h"

// Global mutex for printf
mutex_t printf_mutex;

// Thread-safe printf wrapper
#define safe_printf(...) do { \
    mutex_enter_blocking(&printf_mutex); \
    printf(__VA_ARGS__); \
    mutex_exit(&printf_mutex); \
} while(0)

// Shared data between cores (protected by spin lock in GPS module)
static volatile bool core1_running = false;

// GPS telemetry packet structure (24 bytes now)
typedef struct __attribute__((packed)) {
    uint32_t magic;         // 4 bytes - e.g., 0xFS26 or 0x46533236
    float    latitude;      // 4 bytes
    float    longitude;     // 4 bytes
    float    speed_kph;     // 4 bytes
    float    altitude;      // 4 bytes
    uint16_t tx_count;      // 2 bytes
    uint8_t  satellites;    // 1 byte
    uint8_t  fix_valid;     // 1 byte
} gps_telemetry_packet_t;

// Core 1 entry point - LoRa GPS telemetry broadcast
void core1_main() {
    safe_printf("Core 1: Initializing LoRa TX...\n");
    core1_running = true;
    
    // Initialize LoRa once
    lora_tx_init();
    
    safe_printf("Core 1: Starting GPS telemetry broadcast...\n");
    
    while (true) {
        // Get thread-safe copy of GPS data
        gps_data_t gps;
        gps_get_data_safe(&gps);
        
        // Build telemetry packet
        gps_telemetry_packet_t packet;
        packet.magic      = 0x46533236;  // "FS26" in ASCII hex
        packet.latitude   = (float)gps.raw_latitude;
        packet.longitude  = (float)gps.raw_longitude;
        packet.speed_kph  = (float)gps.speed_kph;
        packet.altitude   = (float)gps.altitude;
        packet.tx_count   = (uint16_t)lora_get_tx_count();
        packet.satellites = (uint8_t)gps.satellites;
        packet.fix_valid  = gps.fix_valid ? 1 : 0;
        
        // Send it (blocking)
        if (lora_send((uint8_t*)&packet, sizeof(packet))) {
            safe_printf("[TX] %.6f, %.6f | %.1f kph | Sats:%d | #%u\n",
                   packet.latitude, packet.longitude, packet.speed_kph,
                   packet.satellites, packet.tx_count);
        } else {
            safe_printf("[TX] FAILED #%lu\n", lora_get_tx_count());
        }
        
        sleep_ms(1000);  // TX rate: 1Hz
    }
}

int main() {
    stdio_init_all();
    mutex_init(&printf_mutex);  // Initialize mutex before anything else
    sleep_ms(2000); 
    
    safe_printf("Core 0: Initializing dual-core GPS + LoRa DAQ system...\n");
    
    // Initialize GPS module on core 0
    gps_init();
    
    // Launch core 1 for LR1121 ping-pong
    safe_printf("Core 0: Launching Core 1 for LR1121 ping-pong...\n");
    multicore_launch_core1(core1_main);
    
    // Wait for core 1 to be ready
    while (!core1_running) {
        sleep_ms(10);
    }
    
    safe_printf("Core 0: Both cores running. Starting GPS processing...\n");
    
    // Core 0 main loop - dedicated GPS processing
    while (true) {
        gps_process();
        
        // Optional: Add small delay to prevent overwhelming the system
        sleep_us(100);
    }
}