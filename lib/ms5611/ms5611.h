#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "../printf/retarget.h"
#include <math.h>

// D1 means preasure
#define MS5611_D1_CONVERSION_OSR_256 0x40
#define MS5611_D1_CONVERSION_OSR_512 0x42
#define MS5611_D1_CONVERSION_OSR_1024 0x44
#define MS5611_D1_CONVERSION_OSR_2048 0x46
#define MS5611_D1_CONVERSION_OSR_4096 0x48

// D2 means temperature
#define MS5611_D2_CONVERSION_OSR_256  0x50
#define MS5611_D2_CONVERSION_OSR_512  0x52
#define MS5611_D2_CONVERSION_OSR_1024 0x54
#define MS5611_D2_CONVERSION_OSR_2048 0x56
#define MS5611_D2_CONVERSION_OSR_4096 0x58

uint8_t init_ms5611(I2C_HandleTypeDef *i2c_handle);

HAL_StatusTypeDef ms5611_reset();

HAL_StatusTypeDef ms5611_start_conversion(uint8_t conversion_type);

void ms5611_set_prefered_data_conversion_temperature(uint8_t prefered_conversion_temperature);
void ms5611_set_prefered_data_conversion_preasure(uint8_t prefered_conversion_preasure);
void ms5611_initiate_prefered_temperature_conversion();
void ms5611_initiate_prefered_preasure_conversion();

float ms5611_read_conversion_temperature_celsius();
float ms5611_read_conversion_preasure_hPa();
float ms5611_read_existing_conversion_preasure_hPa();
uint32_t ms5611_conversion_wait_time_microseconds();

float ms5611_get_height_meters_above_sea_level(uint8_t use_existing_pressure, uint8_t call_for_conversion, float pressure_sea_level_hpa, float temperature_sea_level);
float ms5611_get_height_meters_from_reference(uint8_t use_existing_pressure, uint8_t call_for_conversion, uint8_t reset_reference);
float ms5611_get_height_centimeters_from_reference(uint8_t use_existing_pressure, uint8_t call_for_conversion, uint8_t reset_reference);
void ms5611_set_reference_pressure_from_number_of_samples(uint16_t sample_size);