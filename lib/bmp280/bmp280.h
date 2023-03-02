#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "../printf/retarget.h"
#include <math.h>

#define BMP280_I2C_ID (0x76 << 1)
#define ID_REG 0xD0
#define TRIM_REG 0x88
#define CTRL_MEAS_REG 0xF4
#define CONFIG_REG 0xF5
#define PRES_MSB_REG 0xF7
#define TEMP_MSB_REG 0xFA


enum temp_oversampling {
    OS_TEMP_1  = 0b00100000,
    OS_TEMP_2  = 0b01000000,
    OS_TEMP_4  = 0b01100000,
    OS_TEMP_8  = 0b10000000,
    OS_TEMP_16 = 0b10100000
};

enum pres_oversampling {
    OS_PRES_1  = 0b00000100,
    OS_PRES_2  = 0b00001000,
    OS_PRES_4  = 0b00001100,
    OS_PRES_8  = 0b00010000,
    OS_PRES_16 = 0b00010100
};

enum power_modes {
    SLEEP_MODE  = 0b00000000,
    FORCED_MODE = 0b00000001,
    NORMAL_MODE = 0b00000011,
};

enum standby_modes {
    SB_MODE_0_5  = 0b00000000,
    SB_MODE_62_5 = 0b00000001,
    SB_MODE_125  = 0b00000010,
    SB_MODE_250  = 0b00000011,
    SB_MODE_500  = 0b00000100,
    SB_MODE_1000 = 0b00000101,
    SB_MODE_2000 = 0b00000110,
    SB_MODE_4000 = 0b00000111,
};

enum filter_modes {
    FILTER_MODE_1  = 0b00000000,
    FILTER_MODE_2  = 0b00000100,
    FILTER_MODE_4  = 0b00001000,
    FILTER_MODE_8  = 0b00001100,
    FILTER_MODE_16 = 0b00010000,
};

// Standby modes
#define MODE_0_5

uint8_t init_bmp280(I2C_HandleTypeDef *i2c_address_temp);
float bmp280_pressure_float();
float bmp280_temperature_float();