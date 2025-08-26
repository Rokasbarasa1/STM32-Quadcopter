#pragma once
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <math.h>

uint8_t ist8310_init(
    I2C_HandleTypeDef *i2c_handle_temp, 
    uint8_t apply_calibration, 
    const float hard_iron[3], 
    const float soft_iron[3][3]
);
void ist8310_magnetometer_readings_micro_teslas(float *data, uint8_t perfrom_temperature_correction);
void ist8310_magnetometer_initiate_reading();
void ist8310_magnetometer_readings_micro_teslas_poll(float *data, uint8_t perfrom_temperature_correction);
void ist8310_previous_raw_magetometer_readings(float *data);
float ist8310_previous_temperature_reading();
void ist8310_read_all_otp_data();