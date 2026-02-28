#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "../printf/retarget.h"
#include "../utils/string_utils/string_utils.h"
#include <stdbool.h>

uint8_t init_bn357(UART_HandleTypeDef *uart_temp, DMA_HandleTypeDef *hdma_uart_rx_temp, uint8_t minimal_gps_parse_enabled);
void bn357_toggle_gps_logging(uint8_t status);
void bn357_start_uart_interrupt();
uint8_t bn357_get_status_up_to_date(uint8_t reset_afterwards);
void bn357_get_clear_status();
uint8_t bn357_parse_and_store(char *gps_output_buffer, uint16_t size_of_buf);
float bn357_get_latitude_decimal_format();
float bn357_get_latitude();
char bn357_get_latitude_direction();
float bn357_get_longitude_decimal_format();
float bn357_get_longitude();
float bn357_get_linear_longitude_decimal_format();
char bn357_get_longitude_direction();
float bn357_get_altitude_meters();
float bn357_get_geoid_altitude_meters();
float bn357_get_accuracy();
uint8_t bn357_get_satellites_quantity();
uint8_t bn357_get_fix_quality();
uint8_t bn357_get_utc_time_hours();
uint8_t bn357_get_utc_time_minutes();
uint8_t bn357_get_utc_time_seconds();
uint8_t bn357_get_utc_time_raw();
uint8_t bn357_get_fix_type();
uint8_t bn357_parse_data();
float bn357_get_speed_over_ground_knots();
bool bn357_get_speed_over_ground_knots_stale();
float bn357_get_speed_over_ground_ms();
bool bn357_get_speed_over_ground_ms_stale();
float bn357_get_course_over_ground();
bool bn357_get_course_over_ground_stale();
uint8_t bn357_get_date_day();
uint8_t bn357_get_date_month();
uint8_t bn357_get_date_two_digits_year();
uint16_t bn357_get_date_full_year();
uint16_t bn357_get_GNGGA_string(uint8_t* buffer, uint16_t buffer_size);
bool bn357_GNGGA_string_stale();
uint16_t bn357_get_GNRMC_string(uint8_t* buffer, uint16_t buffer_size);
bool bn357_GNRMC_string_stale();