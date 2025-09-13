#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "../printf/retarget.h"
#include <math.h>

uint8_t init_qmc5883l(I2C_HandleTypeDef *i2c_handle_temp, uint8_t apply_calibration, const float hard_iron[3], const float soft_iron[3][3]);
void qmc5883l_magnetometer_readings_micro_teslas(float *data);
void calculate_yaw_using_magnetometer_data(float *magnetometer_data, float *yaw, float yaw_offset);
void calculate_yaw_tilt_compensated_using_magnetometer_data(float *magnetometer_data, float *yaw_output, float roll, float pitch, float yaw_offset);
void rotate_magnetometer_output_90_degrees_anti_clockwise(float *magnetometer_data);
void rotate_magnetometer_output(float *mag_data, float angle_deg);
void rotate_magnetometer_vector(float *mag, float roll_deg, float pitch_deg, float yaw_deg);
void qmc5883l_previous_raw_magetometer_readings(float *data);

void calculate_yaw_tilt_compensated_using_magnetometer_data_virtual(
    float *magnetometer_data, 
    float *yaw_output, 
    float roll, 
    float pitch, 
    float yaw_offset_degrees,      // final yaw offset to apply
    float virtual_yaw_rotation_degrees // virtual yaw rotation applied before calculation
);
void rotate_magnetometer_data_90(float data[3]);
void rotate_magnetometer_data_180(float data[3]);
void rotate_magnetometer_data_270(float data[3]);

void build_rotation_matrix(float roll_deg, float pitch_deg, float yaw_deg, float R[3][3]);
void rotate_magnetometer_inplace(const float R[3][3], float mag[3]) ;
float calculate_mag_offset_using_compass_rpm(float average_rpm, const float* rpm_values, const float* offset_values, uint8_t length);
