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

uint8_t init_mpu6050(I2C_HandleTypeDef *i2c_address_temp, uint8_t apply_calibration, float accelerometer_correction[3], float gyro_correction[3]);
void mpu6050_accelerometer_readings_float(float *data);
void mpu6050_gyro_readings_float(float *data);
void calculate_pitch_and_roll(float *data, float *roll, float *pitch);
void find_accelerometer_error(uint64_t sample_size);
void find_gyro_error(uint64_t sample_size);