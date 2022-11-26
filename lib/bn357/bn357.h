#pragma once

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "../printf/retarget.h"
#include <stdlib.h>
#include "../utils/string_utils/string_utils.h"

uint8_t init_bn357(UART_HandleTypeDef *uart_temp);
void bn357_parse_and_store(uint8_t *buffer, uint16_t size_of_buf);
float bn357_get_longitude();
float bn357_get_latitude();
char bn357_get_longitude_direction();
char bn357_get_latitude_direction();