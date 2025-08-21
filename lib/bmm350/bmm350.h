#pragma once
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <math.h>

uint8_t bmm350_init(
    I2C_HandleTypeDef *i2c_handle_temp, 
    uint8_t apply_calibration, 
    const float hard_iron[3], 
    const float soft_iron[3][3]
);
void bmm350_magnetometer_readings_micro_teslas(float *data, uint8_t perfrom_temperature_correction);
void bmm350_previous_raw_magetometer_readings(float *data);
float bmm350_previous_temperature_reading();
void bmm350_read_all_otp_data();