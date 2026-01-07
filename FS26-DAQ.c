#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "gps.h"

// Shared data between cores (protected by critical sections if needed)
static volatile bool core1_running = false;
static volatile uint32_t core1_counter = 0;

// Core 1 entry point - placeholder processing
void core1_main() {
    printf("Core 1: Starting placeholder processing...\n");
    core1_running = true;
    
    uint32_t last_log_time = 0;
    
    while (true) {
        // Placeholder processing - simulate data analysis/logging
        core1_counter++;
        
        // Example: Access GPS data from the other core
        const gps_data_t* gps_data = gps_get_data();
        
        // Log core 1 activity every 5 seconds
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_log_time > 5000) {
            printf("Core 1: Processing cycle %u | GPS Fix: %s | Sats: %d\n", 
                   core1_counter, 
                   gps_data->fix_valid ? "YES" : "NO",
                   gps_data->satellites);
            last_log_time = current_time;
        }
        
        // Simulate some processing work
        sleep_ms(100);
        
        // TODO: Add your data processing, logging, calculations, etc. here
        // Examples:
        // - Data logging to SD card
        // - Sensor data fusion
        // - Communication protocols
        // - Real-time calculations
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000); 
    
    printf("Core 0: Initializing dual-core GPS DAQ system...\n");
    
    // Initialize GPS module on core 0
    gps_init();
    
    // Launch core 1 for parallel processing
    printf("Core 0: Launching Core 1 for data processing...\n");
    multicore_launch_core1(core1_main);
    
    // Wait for core 1 to be ready
    while (!core1_running) {
        sleep_ms(10);
    }
    
    printf("Core 0: Both cores running. Starting GPS processing...\n");
    
    // Core 0 main loop - dedicated GPS processing
    while (true) {
        gps_process();
        
        // Optional: Add small delay to prevent overwhelming the system
        sleep_us(100); // Uncomment if needed
    }
}