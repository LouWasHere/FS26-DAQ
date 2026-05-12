/**
 * @file      can_handler.c
 * @brief     CAN bus handler implementation
 */

#include "can_handler.h"
#include "src/mcp2515/MCP2515/MCP2515.h"
#include <stdio.h>

// Global state
static ft550_sensor_data_t g_sensor_data;
static spin_lock_t* g_spin_lock;
static uint32_t g_frame_count = 0;

// FT550 frame IDs we want to receive
static const uint32_t FT550_FRAME_IDS[] = {
    FT550_FRAME_TPS_MAP_TEMPS,
    FT550_FRAME_PRESSURES_GEAR,
    FT550_FRAME_O2_RPM_TEMPS,
    FT550_FRAME_WHEEL_SPEEDS,
    FT550_FRAME_TRACTION_HEADING,
    FT550_FRAME_SHOCK_SENSORS,
    FT550_FRAME_G_FORCE_YAW,
    FT550_FRAME_FUEL_LAMBDA,
    FT550_FRAME_TRANS_TEMPS_FUEL
};

void can_init(void) {
    // Initialize sensor data
    ft550_init_sensor_data(&g_sensor_data);
    
    // Create spin lock for thread-safe access
    g_spin_lock = spin_lock_instance(spin_lock_claim_unused(true));
    
    // Initialize MCP2515
    MCP2515_Init();
    
    printf("CAN: Initialized MCP2515 at 1 Mbps, extended 29-bit identifiers\n");
}

bool can_process_frame(void) {
    uint8_t rx_buffer[8];
    
    // Try to receive each FT550 frame
    for (size_t i = 0; i < sizeof(FT550_FRAME_IDS) / sizeof(FT550_FRAME_IDS[0]); i++) {
        uint32_t frame_id = FT550_FRAME_IDS[i];
        
        // MCP2515_Receive is blocking/polling - it returns if frame available or timeout
        // We'll use a non-blocking approach by checking register directly
        // For now, attempt receive for each known ID
        MCP2515_Receive(frame_id, rx_buffer);
        
        // Try to decode frame
        ft550_sensor_data_t temp_data;
        if (ft550_decode_frame(frame_id, rx_buffer, &temp_data)) {
            // Frame was valid - update global data with spin lock protection
            uint32_t lock_owner = spin_lock_blocking(g_spin_lock);
            {
                // Copy the updated field(s) from temp_data to g_sensor_data
                // Note: ft550_decode_frame only updates relevant fields, so we can selectively copy
                switch (frame_id) {
                    case FT550_FRAME_TPS_MAP_TEMPS:
                        g_sensor_data.tps = temp_data.tps;
                        g_sensor_data.map = temp_data.map;
                        g_sensor_data.air_temp = temp_data.air_temp;
                        g_sensor_data.engine_temp = temp_data.engine_temp;
                        break;
                    case FT550_FRAME_PRESSURES_GEAR:
                        g_sensor_data.oil_pressure = temp_data.oil_pressure;
                        g_sensor_data.fuel_pressure = temp_data.fuel_pressure;
                        g_sensor_data.water_pressure = temp_data.water_pressure;
                        g_sensor_data.gear = temp_data.gear;
                        break;
                    case FT550_FRAME_O2_RPM_TEMPS:
                        g_sensor_data.exhaust_o2 = temp_data.exhaust_o2;
                        g_sensor_data.rpm = temp_data.rpm;
                        g_sensor_data.oil_temp = temp_data.oil_temp;
                        g_sensor_data.pit_limit = temp_data.pit_limit;
                        break;
                    case FT550_FRAME_WHEEL_SPEEDS:
                        g_sensor_data.wheel_speed_fr = temp_data.wheel_speed_fr;
                        g_sensor_data.wheel_speed_fl = temp_data.wheel_speed_fl;
                        g_sensor_data.wheel_speed_rr = temp_data.wheel_speed_rr;
                        g_sensor_data.wheel_speed_rl = temp_data.wheel_speed_rl;
                        break;
                    case FT550_FRAME_TRACTION_HEADING:
                        g_sensor_data.traction_ctrl_slip = temp_data.traction_ctrl_slip;
                        g_sensor_data.traction_ctrl_retard = temp_data.traction_ctrl_retard;
                        g_sensor_data.traction_ctrl_cut = temp_data.traction_ctrl_cut;
                        g_sensor_data.heading = temp_data.heading;
                        break;
                    case FT550_FRAME_SHOCK_SENSORS:
                        g_sensor_data.shock_fr = temp_data.shock_fr;
                        g_sensor_data.shock_fl = temp_data.shock_fl;
                        g_sensor_data.shock_rr = temp_data.shock_rr;
                        g_sensor_data.shock_rl = temp_data.shock_rl;
                        break;
                    case FT550_FRAME_G_FORCE_YAW:
                        g_sensor_data.g_force_accel = temp_data.g_force_accel;
                        g_sensor_data.g_force_lateral = temp_data.g_force_lateral;
                        g_sensor_data.yaw_rate_frontal = temp_data.yaw_rate_frontal;
                        g_sensor_data.yaw_rate_lateral = temp_data.yaw_rate_lateral;
                        break;
                    case FT550_FRAME_FUEL_LAMBDA:
                        g_sensor_data.lambda_correction = temp_data.lambda_correction;
                        g_sensor_data.fuel_flow_total = temp_data.fuel_flow_total;
                        g_sensor_data.inj_time_bank_a = temp_data.inj_time_bank_a;
                        g_sensor_data.inj_time_bank_b = temp_data.inj_time_bank_b;
                        break;
                    case FT550_FRAME_TRANS_TEMPS_FUEL:
                        g_sensor_data.trans_oil_temp = temp_data.trans_oil_temp;
                        g_sensor_data.trans_temp = temp_data.trans_temp;
                        g_sensor_data.fuel_consumption = temp_data.fuel_consumption;
                        g_sensor_data.brake_pressure = temp_data.brake_pressure;
                        break;
                }
                g_frame_count++;
            }
            spin_lock_unsafe_blocking_exit(g_spin_lock, lock_owner);
            
            return true;
        }
    }
    
    return false;
}

void can_get_sensor_data_safe(ft550_sensor_data_t* sensor_data) {
    if (!sensor_data) {
        return;
    }
    
    uint32_t lock_owner = spin_lock_blocking(g_spin_lock);
    {
        *sensor_data = g_sensor_data;
    }
    spin_lock_unsafe_blocking_exit(g_spin_lock, lock_owner);
}

uint32_t can_get_frame_count(void) {
    return g_frame_count;
}
