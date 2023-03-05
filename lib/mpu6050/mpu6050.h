#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "../printf/retarget.h"
#include <math.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define MPU6050 (0b1101000 << 1)
#define I2C_MASTER_TIMEOUT_MS 1000
#define ID_REG 0x75
#define ID_VALUE 104
#define PWR_MGMT_REG 0x6B
#define ACCEL_XOUT_H_REG 0x3B
#define GYRO_XOUT_H_REG 0x43

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
void find_accelerometer_error(uint64_t sample_size);
void find_gyro_error(uint64_t sample_size);