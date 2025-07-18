#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "../printf/retarget.h"
#include <math.h>

enum t_mpu6050_accel_config {
    ACCEL_CONFIG_RANGE_2G    = 0b00000000,
    ACCEL_CONFIG_RANGE_4G    = 0b00001000,
    ACCEL_CONFIG_RANGE_8G    = 0b00010000,
    ACCEL_CONFIG_RANGE_16G   = 0b00011000,
};

enum t_mpu6050_gyro_config {
    GYRO_CONFIG_RANGE_250_DEG    = 0b00000000,
    GYRO_CONFIG_RANGE_500_DEG    = 0b00001000,
    GYRO_CONFIG_RANGE_1000_DEG   = 0b00010000,
    GYRO_CONFIG_RANGE_2000_DEG   = 0b00011000,
};

enum t_mpu6050_low_pass_filter {
    LOW_PASS_FILTER_FREQUENCY_CUTOFF_GYRO_260HZ_ACCEL_256HZ = 0b00000000, // 0ms delay
    LOW_PASS_FILTER_FREQUENCY_CUTOFF_GYRO_184HZ_ACCEL_188HZ = 0b00000001, // 2ms delay
    LOW_PASS_FILTER_FREQUENCY_CUTOFF_GYRO_94HZ_ACCEL_98HZ   = 0b00000010, // 3ms delay
    LOW_PASS_FILTER_FREQUENCY_CUTOFF_GYRO_44HZ_ACCEL_42HZ   = 0b00000011, // 5ms delay
    LOW_PASS_FILTER_FREQUENCY_CUTOFF_GYRO_21HZ_ACCEL_20HZ   = 0b00000100, // 8.5ms delay
    LOW_PASS_FILTER_FREQUENCY_CUTOFF_GYRO_10HZ_ACCEL_10HZ   = 0b00000101, // 14ms delay
    LOW_PASS_FILTER_FREQUENCY_CUTOFF_GYRO_5HZ_ACCEL_5HZ     = 0b00000110, // 19ms delay
};

uint8_t init_mpu6050(I2C_HandleTypeDef *i2c_handle_temp, enum t_mpu6050_accel_config accelerometer_range, enum t_mpu6050_gyro_config gyro_range, enum t_mpu6050_low_pass_filter low_pass_setting, uint8_t apply_calibration, float accelerometer_scale_factor_correction[3][3], float accelerometer_correction[3], float gyro_correction[3], float refresh_rate_hz, float complementary_ratio, float complementary_beta);
void mpu6050_get_accelerometer_readings_gravity(float *data);
void mpu6050_get_gyro_readings_dps(float *data);
void mpu6050_apply_calibrations(float accelerometer_correction[3], float gyro_correction[3]);
void mpu6050_apply_calibrations_gyro(float gyro_correction[3]);
void mpu6050_apply_calibration_accelerometers(float accelerometer_correction[3]);
float mpu6050_calculate_vertical_speed(float last_vertical_speed, float acceleration_data[3], float gyro_degrees[3], int64_t time);
float old_mpu6050_calculate_vertical_acceleration_cm_per_second(float acceleration_data[3], float gyro_degrees[3]);
float mpu6050_calculate_vertical_acceleration_cm_per_second(float acceleration_data[3], float gyro_degrees[3]);

void calculate_roll_pitch_from_accelerometer_data(float *data, float *accelerometer_roll, float *accelerometer_pitch, float roll_offset, float pitch_offset);

void sensor_fusion_roll_pitch(float* gyro_angular, float accelerometer_roll, float accelerometer_pitch, int64_t time, uint8_t set_timestamp, float* imu_orientation);
void sensor_fusion_yaw(float* gyro_angular, float magnetometer_yaw, int64_t time, uint8_t set_timestamp, float* imu_orientation, float* gyro_yaw);



void find_accelerometer_error(uint64_t sample_size);
void find_accelerometer_error_with_corrections(uint64_t sample_size);
void find_gyro_error(uint64_t sample_size);

float angle_difference(float a, float b);
void find_and_return_gyro_error(uint64_t sample_size, float *return_array);

void mpu6050_set_complementary_ratio(float new_complementary_ratio);
void mpu6050_set_complementary_ratio_yaw(float new_complementary_ratio);