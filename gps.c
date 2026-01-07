#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "gps.h"

// --- Private Variables ---
static char nmea_buffer[NMEA_BUFFER_SIZE];
static int buffer_index = 0;
static int total_readings = 0;
static gps_data_t gps_data = {0};

// --- Helper Functions ---

// Custom tokenizer that handles empty fields (e.g. ",,") correctly
static char* nmea_token(char** stringp) {
    char* start = *stringp;
    if (!start) return NULL;
    char* end = strchr(start, ',');
    if (end) {
        *end = '\0';
        *stringp = end + 1;
    } else {
        *stringp = NULL;
    }
    return start;
}

static float nmea_to_decimal(const char* nmea_coord, char direction) {
    if (!nmea_coord || strlen(nmea_coord) == 0) return 0.0;
    float coord = atof(nmea_coord);
    int degrees = (int)(coord / 100);
    float minutes = coord - (degrees * 100);
    float decimal = degrees + (minutes / 60.0);
    if (direction == 'S' || direction == 'W') decimal = -decimal;
    return decimal;
}

static bool verify_nmea_checksum(char* sentence) {
    if (sentence[0] != '$') return false;
    char* asterisk = strrchr(sentence, '*');
    if (asterisk == NULL) return false;
    uint8_t checksum = 0;
    for (char* p = sentence + 1; p < asterisk; p++) checksum ^= *p;
    uint8_t provided_checksum = (uint8_t)strtol(asterisk + 1, NULL, 16);
    return checksum == provided_checksum;
}

// --- NMEA Parsers ---

static void parse_gpgga(char* sentence) {
    char* cursor = sentence;
    nmea_token(&cursor); // Skip tag
    
    int field = 1;
    char* token;
    char lat_str[16]={0}, lat_dir=0, lon_str[16]={0}, lon_dir=0, alt_str[16]={0}, sat_str[8]={0};
    
    while ((token = nmea_token(&cursor)) != NULL && field < 15) {
        switch (field) {
            case 2: strncpy(lat_str, token, 15); break;
            case 3: lat_dir = token[0]; break;
            case 4: strncpy(lon_str, token, 15); break;
            case 5: lon_dir = token[0]; break;
            case 7: strncpy(sat_str, token, 7); break;
            case 8: gps_data.hdop = atof(token); break;
            case 9: strncpy(alt_str, token, 15); break;
        }
        field++;
    }
    
    // Always update satellite count
    if (strlen(sat_str) > 0) gps_data.satellites = atoi(sat_str);

    // Only update position if we actually have data
    if (strlen(lat_str) > 0 && gps_data.satellites > 0) {
        gps_data.fix_valid = true;
        gps_data.raw_latitude = nmea_to_decimal(lat_str, lat_dir);
        gps_data.raw_longitude = nmea_to_decimal(lon_str, lon_dir);
        gps_data.altitude = atof(alt_str);
    } else {
        gps_data.fix_valid = false;
    }
}

static void parse_gprmc(char* sentence) {
    char* cursor = sentence;
    nmea_token(&cursor); // Skip tag
    
    int field = 1;
    char* token;
    char speed_str[16]={0}, course_str[16]={0};
    
    while ((token = nmea_token(&cursor)) != NULL && field < 12) {
        switch (field) {
            case 7: strncpy(speed_str, token, 15); break;
            case 8: strncpy(course_str, token, 15); break;
        }
        field++;
    }
    
    if (strlen(speed_str) > 0) gps_data.speed_kph = atof(speed_str) * 1.852; 
    if (strlen(course_str) > 0) gps_data.course = atof(course_str);
}

// --- Logic Functions ---

static void apply_filtering_and_print() {
    total_readings++;

    // Print raw status even if no fix, so we know it's alive
    if (!gps_data.fix_valid) {
        printf("[%d] Searching... (Sats: %d)\n", total_readings, gps_data.satellites);
        return;
    }

    // Filter 1: Accuracy Check
    if (gps_data.hdop > MAX_HDOP_THRESHOLD) {
        // Signal is too weak/noisy
        return;
    }

    // Filter 2: Stationary Anti-Drift
    if (gps_data.speed_kph >= MIN_SPEED_THRESHOLD) {
        gps_data.is_moving = true;
        gps_data.display_latitude = gps_data.raw_latitude;
        gps_data.display_longitude = gps_data.raw_longitude;
    } else {
        gps_data.is_moving = false;
        // Keep previous display coordinates (locking them)
        // Unless this is the very first reading
        if (gps_data.display_latitude == 0.0) {
            gps_data.display_latitude = gps_data.raw_latitude;
            gps_data.display_longitude = gps_data.raw_longitude;
        }
    }

    printf("[%d] %s | %.6f, %.6f | %.1f kph | 5Hz\n", 
           total_readings,
           gps_data.is_moving ? "MOVING" : "STATIC",
           gps_data.display_latitude,
           gps_data.display_longitude,
           gps_data.speed_kph);
}

static void process_gps_data() {
    for (int i = 0; i < buffer_index; i++) {
        if (nmea_buffer[i] == '\n' || nmea_buffer[i] == '\r') {
            if (i > 0) {
                nmea_buffer[i] = '\0';
                if (verify_nmea_checksum(nmea_buffer)) {
                    if (strncmp(nmea_buffer, "$GPGGA", 6) == 0 || strncmp(nmea_buffer, "$GNGGA", 6) == 0) {
                        parse_gpgga(nmea_buffer);
                    }
                    else if (strncmp(nmea_buffer, "$GPRMC", 6) == 0 || strncmp(nmea_buffer, "$GNRMC", 6) == 0) {
                        parse_gprmc(nmea_buffer);
                        apply_filtering_and_print();
                    }
                }
            }
            int remaining = buffer_index - (i + 1);
            if (remaining > 0) memmove(nmea_buffer, &nmea_buffer[i + 1], remaining);
            buffer_index = remaining;
            i = -1;
        }
    }
}

// --- Public Interface Implementation ---

void gps_init(void) {
    printf("1. Auto-detecting baud rate...\n");
    uart_init(GPS_UART_ID, GPS_TARGET_BAUD);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    
    // Check if we are already fast
    bool already_synced = false;
    absolute_time_t timeout = make_timeout_time_ms(1000);
    while (!time_reached(timeout)) {
        if (uart_is_readable(GPS_UART_ID)) {
            if (uart_getc(GPS_UART_ID) == '$') { already_synced = true; break; }
        }
    }

    if (!already_synced) {
        printf("   - Not synced. Trying cold start sequence (9600 -> 57600)...\n");
        uart_set_baudrate(GPS_UART_ID, 9600);
        sleep_ms(100);
        uart_puts(GPS_UART_ID, GPS_CMD_BAUD);
        sleep_ms(200); 
        uart_set_baudrate(GPS_UART_ID, GPS_TARGET_BAUD);
        sleep_ms(100);
    }

    printf("2. Sending Configuration (5Hz + Optimized Output)...\n");
    // Send multiple times to ensure the module catches it
    for(int i=0; i<3; i++) { 
        uart_puts(GPS_UART_ID, GPS_CMD_SET_OUTPUT); 
        sleep_ms(50); 
    }
    for(int i=0; i<3; i++) { 
        uart_puts(GPS_UART_ID, GPS_CMD_RATE); 
        sleep_ms(50); 
    }
    
    printf(">> GPS Configured. Waiting for Fix...\n");
}

void gps_process(void) {
    while (uart_is_readable(GPS_UART_ID)) {
        char c = uart_getc(GPS_UART_ID);
        if (buffer_index < NMEA_BUFFER_SIZE - 1) nmea_buffer[buffer_index++] = c;
        else buffer_index = 0; 
        if (c == '\n') process_gps_data();
    }
}

bool gps_is_readable(void) {
    return uart_is_readable(GPS_UART_ID);
}

const gps_data_t* gps_get_data(void) {
    return &gps_data;
}