#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "../printf/retarget.h"
#include <math.h>

uint8_t init_mpu6050(I2C_HandleTypeDef *i2c_address_temp, uint8_t apply_calibration, float accelerometer_correction[3], float gyro_correction[3]);
void mpu6050_get_accelerometer_readings_gravity(float *data);
void mpu6050_get_gyro_readings_dps(float *data);
void calculate_pitch_and_roll(float *data, float *roll, float *pitch);
void find_accelerometer_error(uint64_t sample_size);
void find_gyro_error(uint64_t sample_size);