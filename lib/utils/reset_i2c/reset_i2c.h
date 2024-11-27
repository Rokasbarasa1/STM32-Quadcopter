#pragma once

#include "stm32f4xx_hal.h" // Include the HAL library for STM32F411
#include "stdio.h"

void I2C_Bus_Reset(GPIO_TypeDef *GPIO_SCL, uint16_t SCL_Pin, GPIO_TypeDef *GPIO_SDA, uint16_t SDA_Pin);