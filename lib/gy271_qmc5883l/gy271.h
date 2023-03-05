#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "../printf/retarget.h"
#include <math.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define GY271_I2C_ID (0x0D << 1)
#define ID_REG 0x0D
#define ID_VALUE 0b11111111
#define CONTROL1_REG 0x09
#define CONTROL2_REG 0x0A

#define OUTPUT_DATA1_REG 0x00

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

uint8_t init_gy271(I2C_HandleTypeDef *i2c_address_temp, uint8_t apply_calibration, const float hard_iron[3], const float soft_iron[3][3]);
void gy271_magnetometer_readings_micro_teslas(float *data);
void calculate_yaw(float *magnetometer_data, float *yaw);