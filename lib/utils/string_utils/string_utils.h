#pragma once
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "../printf/retarget.h"
#include <stdlib.h>

uint8_t find_substring_from_end(uint8_t *buffer, uint16_t start_index, uint16_t buffer_size, char *string, uint16_t *found_index);
uint8_t find_substring(uint8_t *buffer, uint16_t start_index, uint16_t buffer_size, char *string, uint16_t *found_index);
char *get_substring(uint8_t *buffer, uint16_t buffer_size, uint16_t start_index, uint16_t end_index);
void print_char(uint8_t *data);
void print_binary(uint8_t value);
void print_binary16(uint16_t value);
void print_binary32(uint32_t value);