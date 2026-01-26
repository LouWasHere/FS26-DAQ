#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// --- Configuration ---
#define GPS_UART_ID uart0
#define GPS_TX_PIN 0
#define GPS_RX_PIN 1

// Baud Rate: 57600 is optimal for 5Hz
#define GPS_TARGET_BAUD 57600

// Commands
#define GPS_CMD_RATE        "$PMTK220,200*2C\r\n"   
#define GPS_CMD_BAUD        "$PMTK251,57600*2C\r\n"
// Enable ONLY GGA and RMC to save bandwidth
#define GPS_CMD_SET_OUTPUT  "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"

// Filtering Settings
#define MAX_HDOP_THRESHOLD  3.0f  // Ignore data if accuracy is worse than this
#define MIN_SPEED_THRESHOLD 3.0f  // km/h. Below this, we lock position (Anti-Drift)

// Buffer
#define NMEA_BUFFER_SIZE 256

typedef struct {
    bool fix_valid;
    float raw_latitude;
    float raw_longitude;
    float altitude;
    float speed_kph;
    float course;
    float hdop;
    int satellites;
    
    // Display (Filtered)
    float display_latitude;
    float display_longitude;
    bool is_moving;
} gps_data_t;

// --- Public Interface ---

/**
 * Initialize GPS module with smart baud rate detection and configuration
 */
void gps_init(void);

/**
 * Process any available GPS data from UART
 * Call this regularly in your main loop
 */
void gps_process(void);

/**
 * Check if GPS UART has readable data
 * @return true if data is available
 */
bool gps_is_readable(void);

/**
 * Get current GPS data (read-only access)
 * @return pointer to current GPS data structure
 */
const gps_data_t* gps_get_data(void);

/**
 * Get a thread-safe copy of GPS data
 * Uses spin lock to ensure atomic read across cores
 * @param out Pointer to gps_data_t struct to fill with current data
 */
void gps_get_data_safe(gps_data_t* out);

#endif // GPS_H