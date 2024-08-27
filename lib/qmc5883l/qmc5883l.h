#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "../printf/retarget.h"
#include <math.h>

enum t_interrupts {
    INTERRUPT_PIN_ENABLED  = 0b00000000,
    INTERRUPT_PIN_DISABLED = 0b00000001,
};

enum t_oversampling_ratio {
    OS_RATIO_64   = 0b11000000,
    OS_RATIO_128  = 0b10000000,
    OS_RATIO_256  = 0b01000000,
    OS_RATIO_512  = 0b00000000
};

enum t_mode_control {
    MODE_STANDBY    = 0b00000000,
    MODE_CONTINUOUS = 0b00000001,
};

enum t_output_data_rate {
    ODR_10HZ  = 0b00000000,
    ODR_50HZ  = 0b00000100,
    ODR_100HZ = 0b00001000,
    ODR_200HZ = 0b00001100,
};

enum t_measure_scale {
    MEASURE_SCALE_2G = 0b00000000,
    MEASURE_SCALE_8G = 0b00010000,
};

uint8_t init_qmc5883l(I2C_HandleTypeDef *i2c_handle_temp, uint8_t apply_calibration, const float hard_iron[3], const float soft_iron[3][3]);
void qmc5883l_magnetometer_readings_micro_teslas(float *data);
void calculate_yaw_using_magnetometer_data(float *magnetometer_data, float *yaw);
void calculate_yaw_tilt_compensated_using_magnetometer_data(float *magnetometer_data, float *yaw_output, float roll, float pitch, float yaw_offset);
void rotate_magnetometer_output_90_degrees_anti_clockwise(float *magnetometer_data);