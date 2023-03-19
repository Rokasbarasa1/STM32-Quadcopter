#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "../printf/retarget.h"
#include <stdlib.h>
#include "../utils/string_utils/string_utils.h"

uint8_t init_bn357(UART_HandleTypeDef *uart_temp, uint8_t logging);
uint8_t bn357_get_status_up_to_date(uint8_t reset_afterwards);
void bn357_get_clear_status();
void bn357_parse_and_store(unsigned char *gps_output_buffer, uint16_t size_of_buf);
double bn357_get_latitude_decimal_format();
double bn357_get_longitude_decimal_format();
double bn357_get_altitude_meters();
double bn357_get_geoid_altitude_meters();
double bn357_get_accuracy();
uint8_t bn357_get_satellites_quantity();
uint8_t bn357_get_fix_quality();
uint8_t bn357_get_utc_time_hours();
uint8_t bn357_get_utc_time_minutes();
uint8_t bn357_get_utc_time_seconds();