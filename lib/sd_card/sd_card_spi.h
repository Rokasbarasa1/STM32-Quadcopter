#pragma once

#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include <stdint.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"

#define SD_BUFFER_SIZE 1024

enum t_logger_commands {
    LOGGER_SD_BUFFER_CLEAR    = 0x02 + 16,
    LOGGER_SD_CARD_INITIALIZE    = 0x03 + 16,
    LOGGER_SD_OPEN_FILE    = 0x04 + 16,
    LOGGER_SD_WRITE_DATA_TO_FILE    = 0x05 + 16,
    LOGGER_SD_READ_DATA_FORM_FILE    = 0x06 + 16,
    LOGGER_SD_SET_FILE_CURSOR_OFFSET    = 0x07 + 16,
    LOGGER_SD_CLOSE_FILE    = 0x08 + 16,
    LOGGER_SD_CARD_DEINITIALIZE    = 0x09 + 16,
    LOGGER_SD_CARD_APPEND_TO_BUFFER    = 0x0A + 16,
    LOGGER_SD_GET_BUFFER_POINTER    = 0x0B + 16,
    LOGGER_SD_GET_SELECTED_FILE_SIZE    = 0x0C + 16,
    LOGGER_SD_WRITE_BUFFER_TO_FILE    = 0x0D + 16,
    LOGGER_SD_FILE_EXISTS    = 0x0E + 16,
    LOGGER_SD_SAVE_FILE    = 0x0F + 16,
    LOGGER_SD_BUFFER_SIZE    = 0x11 + 16,

    LOGGER_INITIALIZE    = 0x12 + 16,
    LOGGER_RESET    = 0x13 + 16,
    LOGGER_WRITE_CHUNK_OF_DATA = 0x14 + 16,
    LOGGER_CHECK_READY = 0x15 + 16,

    LOGGER_TEST_INTERFACE = 0x16 + 16,
};


#define LOGGER_INTERFACE_TEST_VALUE 0b01101010
// If someone is using this library then they dont have access to fatfs 
// So this is provided
#define	FA_READ				0x01
#define	FA_WRITE			0x02
#define	FA_OPEN_EXISTING	0x00
#define	FA_CREATE_NEW		0x04
#define	FA_CREATE_ALWAYS	0x08
#define	FA_OPEN_ALWAYS		0x10
#define	FA_OPEN_APPEND		0x30

uint8_t wait_for_slave_ready(uint16_t timeout_ms);
uint8_t sd_card_initialize_spi(SPI_HandleTypeDef * device_handle, GPIO_TypeDef* slave_select_port, uint16_t slave_select_pin, GPIO_TypeDef* slave_ready_port, uint16_t slave_ready_pin);
uint16_t sd_buffer_size(void);
void sd_buffer_clear(void);
uint8_t sd_card_initialize();
uint8_t sd_open_file(const char *file_name, uint8_t instruction);
uint8_t sd_write_data_to_file(const char *data);
uint8_t sd_read_data_from_file();
uint8_t sd_set_file_cursor_offset(uint32_t cursor);
uint8_t sd_close_file();
uint8_t sd_card_deinitialize();
void sd_card_append_to_buffer(const char *string_format, ...);
char* sd_card_get_buffer_pointer();
uint32_t sd_card_get_selected_file_size();
uint8_t sd_write_buffer_to_file();
uint8_t sd_file_exists(const char *file_name);
uint8_t sd_save_file();
uint8_t sd_test_interface();