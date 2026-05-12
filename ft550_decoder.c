/**
 * @file      ft550_decoder.c
 * @brief     FT550 CAN packet decoder implementation
 */

#include "ft550_decoder.h"
#include <string.h>

void ft550_init_sensor_data(ft550_sensor_data_t* sensor_data) {
    if (sensor_data) {
        memset(sensor_data, 0, sizeof(ft550_sensor_data_t));
    }
}

bool ft550_decode_frame(uint32_t frame_id, const uint8_t* data, 
                        ft550_sensor_data_t* sensor_data) {
    if (!data || !sensor_data) {
        return false;
    }

    int16_t raw_m1, raw_m2, raw_m3, raw_m4;

    switch (frame_id) {
        // Frame 0x14080600: TPS, MAP, Air Temp, Engine Temp
        case FT550_FRAME_TPS_MAP_TEMPS:
            raw_m1 = ft550_extract_int16_be(data, 0);
            raw_m2 = ft550_extract_int16_be(data, 2);
            raw_m3 = ft550_extract_int16_be(data, 4);
            raw_m4 = ft550_extract_int16_be(data, 6);
            
            sensor_data->tps = raw_m1 * 0.1f;
            sensor_data->map = raw_m2 * 0.001f;
            sensor_data->air_temp = raw_m3 * 0.1f;
            sensor_data->engine_temp = raw_m4 * 0.1f;
            return true;

        // Frame 0x14080601: Oil Pressure, Fuel Pressure, Water Pressure, Gear
        case FT550_FRAME_PRESSURES_GEAR:
            raw_m1 = ft550_extract_int16_be(data, 0);
            raw_m2 = ft550_extract_int16_be(data, 2);
            raw_m3 = ft550_extract_int16_be(data, 4);
            raw_m4 = ft550_extract_int16_be(data, 6);
            
            sensor_data->oil_pressure = raw_m1 * 0.001f;
            sensor_data->fuel_pressure = raw_m2 * 0.001f;
            sensor_data->water_pressure = raw_m3 * 0.001f;
            sensor_data->gear = raw_m4;
            return true;

        // Frame 0x14080602: Exhaust O2, RPM, Oil Temp, Pit Limit
        case FT550_FRAME_O2_RPM_TEMPS:
            raw_m1 = ft550_extract_int16_be(data, 0);
            raw_m2 = ft550_extract_int16_be(data, 2);
            raw_m3 = ft550_extract_int16_be(data, 4);
            raw_m4 = ft550_extract_int16_be(data, 6);
            
            sensor_data->exhaust_o2 = raw_m1 * 0.001f;
            sensor_data->rpm = (uint16_t)raw_m2;  // RPM: raw × 1
            sensor_data->oil_temp = raw_m3 * 0.1f;
            sensor_data->pit_limit = raw_m4;
            return true;

        // Frame 0x14080603: Wheel Speeds (FR, FL, RR, RL)
        case FT550_FRAME_WHEEL_SPEEDS:
            sensor_data->wheel_speed_fr = ft550_extract_uint16_be(data, 0);
            sensor_data->wheel_speed_fl = ft550_extract_uint16_be(data, 2);
            sensor_data->wheel_speed_rr = ft550_extract_uint16_be(data, 4);
            sensor_data->wheel_speed_rl = ft550_extract_uint16_be(data, 6);
            return true;

        // Frame 0x14080604: Traction Control & Heading
        case FT550_FRAME_TRACTION_HEADING:
            raw_m1 = ft550_extract_int16_be(data, 0);
            raw_m2 = ft550_extract_int16_be(data, 2);
            raw_m3 = ft550_extract_int16_be(data, 4);
            raw_m4 = ft550_extract_int16_be(data, 6);
            
            sensor_data->traction_ctrl_slip = (float)raw_m1;
            sensor_data->traction_ctrl_retard = (float)raw_m2;
            sensor_data->traction_ctrl_cut = (float)raw_m3;
            sensor_data->heading = (float)raw_m4;
            return true;

        // Frame 0x14080605: Shock Sensors (FR, FL, RR, RL)
        case FT550_FRAME_SHOCK_SENSORS:
            raw_m1 = ft550_extract_int16_be(data, 0);
            raw_m2 = ft550_extract_int16_be(data, 2);
            raw_m3 = ft550_extract_int16_be(data, 4);
            raw_m4 = ft550_extract_int16_be(data, 6);
            
            sensor_data->shock_fr = raw_m1 * 0.001f;
            sensor_data->shock_fl = raw_m2 * 0.001f;
            sensor_data->shock_rr = raw_m3 * 0.001f;
            sensor_data->shock_rl = raw_m4 * 0.001f;
            return true;

        // Frame 0x14080606: G-Force & Yaw Rate
        case FT550_FRAME_G_FORCE_YAW:
            raw_m1 = ft550_extract_int16_be(data, 0);
            raw_m2 = ft550_extract_int16_be(data, 2);
            raw_m3 = ft550_extract_int16_be(data, 4);
            raw_m4 = ft550_extract_int16_be(data, 6);
            
            sensor_data->g_force_accel = (float)raw_m1;
            sensor_data->g_force_lateral = (float)raw_m2;
            sensor_data->yaw_rate_frontal = (float)raw_m3;
            sensor_data->yaw_rate_lateral = (float)raw_m4;
            return true;

        // Frame 0x14080607: Lambda Correction, Fuel Flow, Injection Time
        case FT550_FRAME_FUEL_LAMBDA:
            raw_m1 = ft550_extract_int16_be(data, 0);
            raw_m2 = ft550_extract_int16_be(data, 2);
            raw_m3 = ft550_extract_int16_be(data, 4);
            raw_m4 = ft550_extract_int16_be(data, 6);
            
            sensor_data->lambda_correction = (float)raw_m1;
            sensor_data->fuel_flow_total = raw_m2 * 0.01f;
            sensor_data->inj_time_bank_a = raw_m3 * 0.01f;
            sensor_data->inj_time_bank_b = raw_m4 * 0.01f;
            return true;

        // Frame 0x14080608: Transmission & Brake
        case FT550_FRAME_TRANS_TEMPS_FUEL:
            raw_m1 = ft550_extract_int16_be(data, 0);
            raw_m2 = ft550_extract_int16_be(data, 2);
            raw_m3 = ft550_extract_int16_be(data, 4);
            raw_m4 = ft550_extract_int16_be(data, 6);
            
            sensor_data->trans_oil_temp = raw_m1 * 0.1f;
            sensor_data->trans_temp = raw_m2 * 0.1f;
            sensor_data->fuel_consumption = (float)raw_m3;
            sensor_data->brake_pressure = raw_m4 * 0.001f;
            return true;

        default:
            // Unrecognized frame ID
            return false;
    }
}
