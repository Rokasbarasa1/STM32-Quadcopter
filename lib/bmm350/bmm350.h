#pragma once
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <math.h>

enum bmm350_PAD_CTRL_odr {
    BMM350_PAD_CTRL_ODR_400_HZ                               = 0b00000010,
    BMM350_PAD_CTRL_ODR_200_HZ                               = 0b00000011,
    BMM350_PAD_CTRL_ODR_100_HZ                               = 0b00000100,
    BMM350_PAD_CTRL_ODR_50_HZ                                = 0b00000101,
    BMM350_PAD_CTRL_ODR_25_HZ                                = 0b00000110,
    BMM350_PAD_CTRL_ODR_12_5_HZ                              = 0b00000111,
    BMM350_PAD_CTRL_ODR_6_25_HZ                              = 0b00001000,
    BMM350_PAD_CTRL_ODR_3_125_HZ                             = 0b00001001,
    BMM350_PAD_CTRL_ODR_1_5625_HZ                            = 0b00001010,
};

enum bmm350_PAD_CTRL_avg {
    BMM350_PAD_CTRL_AVG_NO_AVG                               = 0b00000000,
    BMM350_PAD_CTRL_AVG_2                                    = 0b00010000,
    BMM350_PAD_CTRL_AVG_4                                    = 0b00100000,
    BMM350_PAD_CTRL_AVG_8                                    = 0b00110000
};

uint8_t bmm350_init(
    I2C_HandleTypeDef *i2c_handle_temp, 
    uint8_t apply_calibration, 
    const float hard_iron[3], 
    const float soft_iron[3][3],
    enum bmm350_PAD_CTRL_odr odr_setting,
    enum bmm350_PAD_CTRL_avg average_setting
);
void bmm350_magnetometer_readings_micro_teslas(float *data, uint8_t perfrom_temperature_correction);
void bmm350_previous_raw_magetometer_readings(float *data);
float bmm350_previous_temperature_reading();
void bmm350_read_all_otp_data();