#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "../printf/retarget.h"
#include <math.h>

enum t_power_management {
    PWR_RESET    = 0b10000000,
    PWR_SLEEP    = 0b01000000,
    PWR_CYCLE    = 0b00100000,
    PWR_TEMP_DIS = 0b00001000,
    PWR_CLOCK_INTERNAL_8MHZ = 0b00000000,
    PWR_CLOCK_INTERNAL_STOP = 0b00000111,
};

uint8_t init_mpu6050(I2C_HandleTypeDef *i2c_address_temp, uint8_t apply_calibration, const float accelerometer_correction[3], const float gyro_correction[3]);
void mpu6050_get_accelerometer_readings_gravity(float *data);
void mpu6050_get_gyro_readings_dps(float *data);
void calculate_pitch_and_roll(float *data, float *roll, float *pitch);
void calculate_degrees_x_y(float *data, float *rotation_around_x, float *rotation_around_y);
void find_accelerometer_error(uint64_t sample_size);
void find_gyro_error(uint64_t sample_size);