#pragma once
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "../printf/retarget.h"
#include <math.h>


uint8_t mmc5603_get_register_value(uint8_t register_address);
uint8_t mmc5603_init(
    I2C_HandleTypeDef *i2c_handle_temp, 
    uint8_t apply_calibration, 
    const float hard_iron[3], 
    const float soft_iron[3][3],
    uint8_t use_continuos_mode,
    uint16_t refresh_rate,
    uint8_t use_automatic_set_reset
);
void mmc5603_magnetometer_readings_micro_teslas(float *data, uint8_t apply_rotation_into_accelerometer_position);
void mmc5603_set();
void mmc5603_reset();
uint8_t mmc5603_read_status_register();
uint8_t mmc5603_perform_self_test();
void mmc5603_get_bridge_offset(float *data);
void mmc5603_magnetometer_readings_micro_teslas_bridge_offset_removed(float *data);
void mmc5603_previous_raw_magetometer_readings(float *data);
void mmc5603_set_rotation_matrix(const float rotation_matrix[3][3]);
void mmc5603_magnetometer_readings_micro_teslas_unrotated(float *data);