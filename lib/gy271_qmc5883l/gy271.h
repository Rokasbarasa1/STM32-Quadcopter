#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_exti.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_uart.h"
#include "../printf/retarget.h"
#include <math.h>

uint8_t init_gy271(I2C_HandleTypeDef *i2c_address_temp, uint8_t apply_calibration, float hard_iron[3], float soft_iron[3][3]);
void gy271_magnetometer_readings_micro_teslas(float *data);
void calculate_yaw(float *magnetometer_data, float *yaw);