#pragma once

#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include <stdint.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"

#define SD_BUFFER_SIZE 10000

enum t_logger_commands {
    LOGGER_SD_BUFFER_CLEAR = 1,
    LOGGER_SD_CARD_INITIALIZE = 2,
    LOGGER_SD_OPEN_FILE = 3,
    LOGGER_SD_WRITE_DATA_TO_FILE = 4,
    LOGGER_SD_READ_DATA_FORM_FILE = 5,
    LOGGER_SD_SET_FILE_CURSOR_OFFSET = 6,
    LOGGER_SD_CLOSE_FILE = 7,
    LOGGER_SD_CARD_DEINITIALIZE = 8,
    LOGGER_SD_CARD_APPEND_TO_BUFFER = 10,
    LOGGER_SD_GET_BUFFER_POINTER = 11,
    LOGGER_SD_GET_SELECTED_FILE_SIZE = 12,
    LOGGER_SD_WRITE_BUFFER_TO_FILE = 13,
    LOGGER_SD_FILE_EXISTS = 14,
    LOGGER_SD_SAVE_FILE = 15,
    LOGGER_SD_BUFFER_SIZE = 16,

    LOGGER_INITIALIZE = 17,
    LOGGER_GET_FILE_INDEX = 27,
    LOGGER_RESET = 18,
    LOGGER_WRITE_CHUNK_OF_STRING_DATA = 20,
    LOGGER_WRITE_CHUNK_OF_BYTE_DATA = 21,
    LOGGER_TEST_INTERFACE = 19,
    LOGGER_TEST_INTERFACE_BROKEN = 9,

    LOGGER_WRITE_CHUNK_OF_STRING_DATA_ASYNC = 22,
    LOGGER_WRITE_CHUNK_OF_BYTE_DATA_ASYNC = 23,

    LOGGER_ENTER_ASYNC_STRING_MODE = 24,
    LOGGER_ENTER_ASYNC_BYTE_MODE = 25,
    LOGGER_LEAVE_ASYNC_MODE = 26
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
uint16_t sd_buffer_size(uint8_t local);
uint8_t sd_buffer_clear_index(uint8_t local);
uint8_t sd_buffer_clear(uint8_t local);
uint8_t sd_card_initialize();
uint8_t sd_open_file(uint8_t *file_name, uint8_t instruction);
uint8_t sd_write_data_to_file(uint8_t *data);
uint8_t sd_read_data_from_file();
uint8_t sd_set_file_cursor_offset(uint32_t cursor);
uint8_t sd_close_file();
uint8_t sd_card_deinitialize();
uint8_t sd_card_append_to_buffer(uint8_t local, const char *string_format, ...);
uint8_t* sd_card_get_buffer_pointer(uint8_t local);
void sd_card_buffer_increment_index();
void sd_card_buffer_increment_index_by_amount(uint16_t index_increment_amount);

uint32_t sd_card_get_selected_file_size();
uint8_t sd_write_buffer_to_file();
uint8_t sd_file_exists(uint8_t *file_name);
uint8_t sd_save_file();
uint8_t sd_test_interface();

uint8_t sd_special_initialize(uint8_t *file_base_name);
uint16_t sd_special_get_file_index();
uint8_t sd_special_reset();
uint8_t sd_special_write_chunk_of_string_data(uint8_t *data);
uint8_t sd_special_write_chunk_of_byte_data(uint8_t *data, uint16_t length);
uint8_t sd_special_enter_async_string_mode(uint8_t reset_sd_initialization_when_async_stops);
uint8_t sd_special_enter_async_byte_mode(uint8_t reset_sd_initialization_when_async_stops);

uint8_t sd_special_leave_async_mode();
uint8_t sd_special_write_chunk_of_string_data_no_slave_response(uint8_t *data);
uint8_t sd_special_write_chunk_of_byte_data_no_slave_response(uint8_t *data, uint16_t length);

uint8_t sd_special_write_chunk_of_string_data_async(uint8_t *data);
uint8_t sd_special_write_chunk_of_byte_data_async(uint8_t *data, uint16_t length);

uint8_t sd_special_wait_until_async_write_done();
void sd_buffer_swap();
uint8_t sd_get_response();
void sd_card_set_dma_transfer_call_status(uint8_t status);
void sd_card_wait_for_dma_transfer_complete();
void sd_card_wait_for_slave_ready();