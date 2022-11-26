#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "../printf/retarget.h"
#include <math.h>

uint8_t init_gy271(I2C_HandleTypeDef *i2c_address_temp, uint8_t apply_calibration, float hard_iron[3], float soft_iron[3][3]);
void gy271_magnetometer_readings_micro_teslas(float *data);
void calculate_yaw(float *magnetometer_data, float *yaw);