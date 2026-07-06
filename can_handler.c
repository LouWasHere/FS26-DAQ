/**
 * @file      can_handler.c
 * @brief     CAN bus handler implementation
 */

#include "can_handler.h"
#include "src/mcp2515/MCP2515/MCP2515.h"
#include "src/mcp2515/Config/DEV_Config.h"
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
    
    // Initialize hardware (SPI, GPIO, etc.) - MUST be called before MCP2515_Init()
    DEV_Module_Init();
    
    // Initialize MCP2515
    MCP2515_Init();
    
    printf("CAN: Initialized MCP2515 at 1 Mbps, extended 29-bit identifiers\n");
}

bool can_process_frame(void) {
    uint32_t received_id = 0;
    uint8_t rx_buffer[8] = {0}; 
    
    if (MCP2515_Receive_Fast(&received_id, rx_buffer) != 0) {
        return false; 
    }

    if (received_id != 0x100) return true; 

    static uint8_t m84_block[256]; // Increased buffer slightly for safety
    static int frame_index = 0;
    static uint32_t last_rx_time = 0;

    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // If there is a gap of >5ms, the previous burst finished. Decode it!
    if ((current_time - last_rx_time) > 5) {
        
        int anchor_idx = -1;
        
        // Search the assembled block for the MoTeC Magic Number: 82 81 80 54
        for (int i = 0; i < (frame_index * 8) - 3; i++) {
            if (m84_block[i] == 0x82 && m84_block[i+1] == 0x81 && 
                m84_block[i+2] == 0x80 && m84_block[i+3] == 0x54) {
                anchor_idx = i;
                break;
            }
        }

        if (anchor_idx != -1) {
            #define MOTEC_I16(offset) ((int16_t)((m84_block[offset] << 8) | m84_block[offset + 1]))

            uint32_t lock_owner = spin_lock_blocking(g_spin_lock);
            {
                // Dynamically map based on the anchor position
                // (anchor_idx is normally 8, but will adapt if frames drop)
                g_sensor_data.tps = MOTEC_I16(anchor_idx + 6) * 0.1f;           
                g_sensor_data.rpm = MOTEC_I16(anchor_idx + 4);                
                g_sensor_data.engine_temp = MOTEC_I16(anchor_idx + 12) * 0.1f;   
                g_sensor_data.air_temp = MOTEC_I16(anchor_idx + 14) * 0.1f;      
                g_sensor_data.battery_voltage = MOTEC_I16(anchor_idx + 48) * 0.01f;
                g_sensor_data.map = MOTEC_I16(anchor_idx + 78) * 0.1f;           
                
                g_frame_count++;
            }
            spin_unlock(g_spin_lock, lock_owner);
        } else {
            // Optional: Print a warning if the block was too corrupt to find the anchor
            // printf("Warning: M84 Magic Number not found in block!\n");
        }
        
        frame_index = 0; 
    }
    
    last_rx_time = current_time;

    // Assemble the frames
    if (frame_index < 32) { // 32 * 8 = 256 bytes max
        memcpy(&m84_block[frame_index * 8], rx_buffer, 8);
        frame_index++;
    }

    return true; 
}

void can_get_sensor_data_safe(ft550_sensor_data_t* sensor_data) {
    if (!sensor_data) {
        return;
    }
    
    uint32_t lock_owner = spin_lock_blocking(g_spin_lock);
    {
        *sensor_data = g_sensor_data;
    }
    spin_unlock(g_spin_lock, lock_owner);
}

uint32_t can_get_frame_count(void) {
    return g_frame_count;
}
