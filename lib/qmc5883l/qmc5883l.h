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