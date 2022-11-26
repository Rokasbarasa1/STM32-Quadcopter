#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "../printf/retarget.h"
#include <math.h>

uint8_t init_bmp280(I2C_HandleTypeDef *i2c_address_temp);
float bmp280_preassure_float();
float bmp280_temperature_float();