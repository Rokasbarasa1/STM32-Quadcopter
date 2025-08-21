#pragma once
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "../printf/retarget.h"
#include <math.h>

void i2c_scan_bus(I2C_HandleTypeDef *hi2c);