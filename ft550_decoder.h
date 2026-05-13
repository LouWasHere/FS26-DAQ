/**
 * @file      ft550_decoder.h
 * @brief     FT550 CAN packet decoder with FTCAN 2.0 protocol multiplier support
 * 
 * Handles decoding of 8 simplified CAN frame types from FT550 ECU,
 * parsing signed 16-bit big-endian values, and applying protocol-defined multipliers.
 */

#ifndef FT550_DECODER_H
#define FT550_DECODER_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/**
 * FT550 CAN Frame IDs (Extended 29-bit)
 */
typedef enum {
    FT550_FRAME_TPS_MAP_TEMPS       = 0x14080600,  // TPS, MAP, Air Temp, Engine Temp
    FT550_FRAME_PRESSURES_GEAR      = 0x14080601,  // Oil Press, Fuel Press, Water Press, Gear
    FT550_FRAME_O2_RPM_TEMPS        = 0x14080602,  // Exhaust O2, RPM, Oil Temp, Pit Limit
    FT550_FRAME_WHEEL_SPEEDS        = 0x14080603,  // Wheel Speed FR, FL, RR, RL
    FT550_FRAME_TRACTION_HEADING    = 0x14080604,  // TC Slip, TC Retard, TC Cut, Heading
    FT550_FRAME_SHOCK_SENSORS       = 0x14080605,  // Shock FR, FL, RR, RL
    FT550_FRAME_G_FORCE_YAW         = 0x14080606,  // G-force accel, lateral, Yaw-rate frontal, lateral
    FT550_FRAME_FUEL_LAMBDA         = 0x14080607,  // Lambda Correction, Fuel Flow, Inj Time A, B
    FT550_FRAME_TRANS_TEMPS_FUEL    = 0x14080608   // Trans Oil Temp, Trans Temp, Fuel Consumption, Brake Press
} ft550_frame_id_t;

/**
 * Combined sensor data from all FT550 frames
 * Stores decoded values with multipliers already applied
 */
typedef struct {
    // Frame 0x14080600 - TPS, MAP, Temps
    float tps;                          // % (Raw × 0.1)
    float map;                          // Bar (Raw × 0.001)
    float air_temp;                     // °C (Raw × 0.1)
    float engine_temp;                  // °C (Raw × 0.1)
    
    // Frame 0x14080601 - Pressures, Gear
    float oil_pressure;                 // Bar (Raw × 0.001)
    float fuel_pressure;                // Bar (Raw × 0.001)
    float water_pressure;               // Bar (Raw × 0.001)
    int16_t gear;                       // Raw (No multiplier)
    
    // Frame 0x14080602 - O2, RPM, Temps
    float exhaust_o2;                   // λ (Raw × 0.001)
    uint16_t rpm;                       // RPM (Raw × 1)
    float oil_temp;                     // °C (Raw × 0.1)
    int16_t pit_limit;                  // Raw (No multiplier)
    
    // Frame 0x14080603 - Wheel Speeds
    uint16_t wheel_speed_fr;            // km/h (Raw × 1)
    uint16_t wheel_speed_fl;            // km/h (Raw × 1)
    uint16_t wheel_speed_rr;            // km/h (Raw × 1)
    uint16_t wheel_speed_rl;            // km/h (Raw × 1)
    
    // Frame 0x14080604 - Traction Control & Heading
    float traction_ctrl_slip;           // Raw (No multiplier)
    float traction_ctrl_retard;         // Raw (No multiplier)
    float traction_ctrl_cut;            // Raw (No multiplier)
    float heading;                      // Raw (No multiplier)
    
    // Frame 0x14080605 - Shock Sensors
    float shock_fr;                     // Raw × 0.001
    float shock_fl;                     // Raw × 0.001
    float shock_rr;                     // Raw × 0.001
    float shock_rl;                     // Raw × 0.001
    
    // Frame 0x14080606 - G-Forces & Yaw Rate
    float g_force_accel;                // Raw (No multiplier, already in g)
    float g_force_lateral;              // Raw (No multiplier, already in g)
    float yaw_rate_frontal;             // Raw (No multiplier)
    float yaw_rate_lateral;             // Raw (No multiplier)
    
    // Frame 0x14080607 - Lambda, Fuel Flow, Injection
    float lambda_correction;            // Raw (No multiplier)
    float fuel_flow_total;              // L/min (Raw × 0.01)
    float inj_time_bank_a;              // ms (Raw × 0.01)
    float inj_time_bank_b;              // ms (Raw × 0.01)
    
    // Frame 0x14080608 - Transmission & Brake
    float trans_oil_temp;               // °C (Raw × 0.1)
    float trans_temp;                   // °C (Raw × 0.1)
    float fuel_consumption;             // L (Raw × 1)
    float brake_pressure;               // Bar (Raw × 0.001)
} ft550_sensor_data_t;

/**
 * @brief Decode FT550 CAN frame and update sensor data
 * 
 * Parses an 8-byte CAN frame according to FT550 format:
 * - Big-endian signed 16-bit integers (INT16)
 * - Applies protocol-defined multipliers
 * - Updates corresponding fields in sensor_data
 * 
 * @param frame_id CAN extended ID (0x14080600-0x14080608)
 * @param data Pointer to 8-byte CAN data payload
 * @param sensor_data Pointer to sensor data structure to update
 * @return true if frame was valid and processed, false if frame ID not recognized
 */
bool ft550_decode_frame(uint32_t frame_id, const uint8_t* data, 
                        ft550_sensor_data_t* sensor_data);

/**
 * @brief Extract signed 16-bit big-endian value from CAN data
 * 
 * @param data Pointer to CAN frame data
 * @param byte_offset Offset in bytes (0-6 for 8-byte frame)
 * @return Signed 16-bit integer value
 */
static inline int16_t ft550_extract_int16_be(const uint8_t* data, uint8_t byte_offset) {
    return (int16_t)((data[byte_offset] << 8) | data[byte_offset + 1]);
}

/**
 * @brief Extract unsigned 16-bit big-endian value from CAN data
 * 
 * @param data Pointer to CAN frame data
 * @param byte_offset Offset in bytes (0-6 for 8-byte frame)
 * @return Unsigned 16-bit integer value
 */
static inline uint16_t ft550_extract_uint16_be(const uint8_t* data, uint8_t byte_offset) {
    return (uint16_t)((data[byte_offset] << 8) | data[byte_offset + 1]);
}

/**
 * @brief Initialize sensor data structure with zero/default values
 * 
 * @param sensor_data Pointer to sensor data structure to initialize
 */
void ft550_init_sensor_data(ft550_sensor_data_t* sensor_data);

#endif // FT550_DECODER_H
