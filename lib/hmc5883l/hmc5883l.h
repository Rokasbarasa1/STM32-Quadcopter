#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "../printf/retarget.h"
#include <math.h>



uint8_t init_hmc5883l(I2C_HandleTypeDef *i2c_handle_temp, uint8_t apply_calibration, const float hard_iron[3], const float soft_iron[3][3]);
void hmc5883l_magnetometer_readings_micro_teslas(float *data);
void hmc5883l_previous_raw_magetometer_readings(float *data);