#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "gps.h"
#include "lr1121_tx.h"
#include "can_handler.h"
#include "ft550_decoder.h"

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

// GPS telemetry packet structure with integrated CAN data
typedef struct __attribute__((packed)) {
    uint32_t magic;         // 4 bytes - 0x46533236 ("FS26")
    
    // GPS Data
    float    latitude;      // 4 bytes
    float    longitude;     // 4 bytes
    float    gps_speed_kph; // 4 bytes
    float    altitude;      // 4 bytes
    uint8_t  satellites;    // 1 byte
    uint8_t  fix_valid;     // 1 byte
    
    // CAN Data - Engine Parameters
    uint16_t rpm;           // 2 bytes - RPM
    float    engine_temp;   // 4 bytes - °C
    float    tps;           // 4 bytes - Throttle Position %
    
    // CAN Data - Pressures & Fluids
    float    oil_pressure;  // 4 bytes - Bar
    float    fuel_pressure; // 4 bytes - Bar
    float    brake_pressure;// 4 bytes - Bar
    float    fuel_flow;     // 4 bytes - L/min
    
    // CAN Data - Wheel Speeds
    uint16_t wheel_speed_fr;// 2 bytes - km/h
    uint16_t wheel_speed_fl;// 2 bytes - km/h
    uint16_t wheel_speed_rr;// 2 bytes - km/h
    uint16_t wheel_speed_rl;// 2 bytes - km/h
    
    // CAN Data - Dynamics
    float    g_force_lateral;// 4 bytes
    float    heading;       // 4 bytes
    
    // Packet Metadata
    uint16_t tx_count;      // 2 bytes - LoRa TX count
    uint16_t can_frame_count;// 2 bytes - CAN frames received
} combined_telemetry_packet_t;

// Core 1 entry point - LoRa broadcast with GPS + CAN telemetry
void core1_main() {
    safe_printf("Core 1: Initializing LoRa TX...\n");
    lora_tx_init();
    
    safe_printf("Core 1: Initializing CAN bus for FT550 data...\n");
    can_init();
    
    core1_running = true;
    
    safe_printf("Core 1: Starting combined telemetry broadcast (GPS + CAN + LoRa)...\n");
    
    while (true) {        
        // Get thread-safe copy of GPS data
        gps_data_t gps;
        gps_get_data_safe(&gps);
        
        // Get thread-safe copy of CAN sensor data
        ft550_sensor_data_t can_data;
        can_get_sensor_data_safe(&can_data);
        
        // Build combined telemetry packet
        combined_telemetry_packet_t packet;
        packet.magic = 0x46533236;  // "FS26" magic number
        
        // GPS Data
        packet.latitude = gps.raw_latitude;
        packet.longitude = gps.raw_longitude;
        packet.gps_speed_kph = gps.speed_kph;
        packet.altitude = gps.altitude;
        packet.satellites = (uint8_t)gps.satellites;
        packet.fix_valid = gps.fix_valid ? 1 : 0;
        
        // CAN Data - Engine Parameters
        packet.rpm = can_data.rpm;
        packet.engine_temp = can_data.engine_temp;
        packet.tps = can_data.tps;
        
        // CAN Data - Pressures & Fluids
        packet.oil_pressure = can_data.oil_pressure;
        packet.fuel_pressure = can_data.fuel_pressure;
        packet.brake_pressure = can_data.brake_pressure;
        packet.fuel_flow = can_data.fuel_flow_total;
        
        // CAN Data - Wheel Speeds
        packet.wheel_speed_fr = can_data.wheel_speed_fr;
        packet.wheel_speed_fl = can_data.wheel_speed_fl;
        packet.wheel_speed_rr = can_data.wheel_speed_rr;
        packet.wheel_speed_rl = can_data.wheel_speed_rl;
        
        // CAN Data - Dynamics
        packet.g_force_lateral = can_data.g_force_lateral;
        packet.heading = can_data.heading;
        
        // Packet Metadata
        packet.tx_count = (uint16_t)lora_get_tx_count();
        packet.can_frame_count = (uint16_t)(can_get_frame_count() & 0xFFFF);
        
        // Send it (blocking)
        if (lora_send((uint8_t*)&packet, sizeof(packet))) {
            safe_printf("[TX] GPS:%.6f,%.6f | RPM:%u | TPS:%.3f | WheelSp(FR/FL/RR/RL):%u/%u/%u/%u | TX#%u CAN#%u\n",
                   packet.latitude, packet.longitude, 
                   packet.rpm, packet.tps,
                   packet.wheel_speed_fr, packet.wheel_speed_fl, 
                   packet.wheel_speed_rr, packet.wheel_speed_rl,
                   packet.tx_count, packet.can_frame_count);
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
    
    // Launch core 1 for LR1121
    safe_printf("Core 0: Launching Core 1 for LR1121 ping-pong...\n");
    multicore_launch_core1(core1_main);
    
    // Wait for core 1 to be ready
    while (!core1_running) {
        sleep_ms(10);
    }
    
    safe_printf("Core 0: Both cores running. Starting GPS processing...\n");
    
    // Core 0 main loop - dedicated GPS & CAN processing
    while (true) {
        // Poll GPS UART
        gps_process();
        
        // DRAIN LOOP: Pull every waiting message out of the MCP2515 
        // until the hardware buffer is completely empty.
        while (can_process_frame()) {
            // Keep looping as long as it returns true!
        }
        
        // Small delay to prevent locking the bus completely
        sleep_us(100);
    }
}