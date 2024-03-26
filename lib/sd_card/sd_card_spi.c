#include "./sd_card_spi.h"

#define SD_CARD_DEBUG 1

uint8_t driver_initialized = 0;

SPI_HandleTypeDef * m_device_handle;
GPIO_TypeDef* m_slave_select_port = GPIOB;
uint16_t m_slave_select_pin = GPIO_PIN_11;

GPIO_TypeDef* m_slave_ready_port = GPIOB;
uint16_t m_slave_ready_pin = GPIO_PIN_11;

#define STRING_BUFFER_SIZE 300
uint8_t string_buffer[STRING_BUFFER_SIZE];

volatile uint8_t slave_ready = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == m_slave_ready_pin && slave_ready == 0 && driver_initialized == 1){
        // Falling 
        slave_ready = 1;
    }
}

void start_waiting_for_slave_ready(){
    slave_ready = 0;
}

uint8_t wait_for_slave_ready(uint16_t timeout_ms) {
    uint32_t start_time = HAL_GetTick();
    uint32_t delta_time = 0;

    // Wait for it to be busy
    while (slave_ready == 0 && (delta_time = (HAL_GetTick() - start_time)) < timeout_ms );

    // // Wait for it to be busy
    // while (HAL_GPIO_ReadPin(m_slave_ready_port, m_slave_ready_pin) == GPIO_PIN_SET && (delta_time = (HAL_GetTick() - start_time)) < timeout_ms );

    // // Wait to not be busy anymore
    // while (HAL_GPIO_ReadPin(m_slave_ready_port, m_slave_ready_pin) == GPIO_PIN_RESET && (delta_time = (HAL_GetTick() - start_time)) < timeout_ms );

    if(delta_time >= timeout_ms){
        return 0;
    }

    return 1;
}

void slave_deselect(){
    HAL_GPIO_WritePin(m_slave_select_port, m_slave_select_pin, GPIO_PIN_SET);
}

void slave_select(){
    HAL_GPIO_WritePin(m_slave_select_port, m_slave_select_pin, GPIO_PIN_RESET);
}

uint8_t sd_card_initialize_spi(SPI_HandleTypeDef * device_handle, GPIO_TypeDef* slave_select_port, uint16_t slave_select_pin, GPIO_TypeDef* slave_ready_port, uint16_t slave_ready_pin){
    if (device_handle){
        m_device_handle = device_handle;
        
        m_slave_select_port = slave_select_port;
        m_slave_select_pin = slave_select_pin;

        m_slave_ready_port = slave_ready_port;
        m_slave_ready_pin = slave_ready_pin;

        driver_initialized = 1;
        return 1;
    }else{
        return 0;
    }
}

uint16_t sd_buffer_size(void){
    if(m_device_handle){
        uint8_t command = LOGGER_SD_BUFFER_SIZE;
        uint8_t response[2];

        HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 2, 5000);
        slave_deselect();
        return (response[0] << 8) | response[1];
    }else{
        return 0;
    }

    error:
        slave_deselect();
        return 0;
}

uint8_t sd_buffer_clear(void){
    if(m_device_handle){
        uint8_t command = LOGGER_SD_BUFFER_CLEAR;
        uint8_t response[1];

        HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();
        return 1;
    }
    return 0;

    error:
        slave_deselect();
        return 0;
}

uint8_t sd_card_initialize(){
    if(m_device_handle){
        uint8_t command = LOGGER_SD_CARD_INITIALIZE;
        uint8_t response[1];

        HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();

        if(!response[0]){
            goto error;
        }

#if(SD_CARD_DEBUG)
        printf("SD card mounted\n");
#endif
        return 1;
    }else{
        return 0;
    }

    error:
#if(SD_CARD_DEBUG)
        printf("SD card not mounted\n");
#endif
        slave_deselect();
        return 0;
}

uint8_t sd_open_file(const char *file_name, uint8_t instruction){
    if(m_device_handle){
        uint8_t response[1];
        uint8_t command = LOGGER_SD_OPEN_FILE;
        uint16_t file_length = strlen(file_name)+1;
        uint16_t total_length = file_length+1;
        uint8_t transmit_length[] = {(total_length >> 8) & 0xFF, total_length & 0xFF};
        uint8_t transmit_buffer[total_length]; // string length, \0 and command
        transmit_buffer[0] = instruction;
        memcpy(&transmit_buffer[1], file_name, file_length); // Copy the string including null terminator.

        printf("open file: data '%s' instruction %d length %d\n", (char*)transmit_buffer, instruction, total_length);
        volatile HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000); // send command
        if(!wait_for_slave_ready(1000)) goto error;
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, transmit_length, 2, 5000); // send how many bytes the dat will be
        if(!wait_for_slave_ready(1000)) goto error;
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, transmit_buffer, total_length, 5000); // send data
        if(!wait_for_slave_ready(1000)) goto error;
        // start_waiting_for_slave_ready();
        status = HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        // if(!wait_for_slave_ready(1000)) goto error;
        // uint8_t data[200];
        // status = HAL_SPI_Receive(m_device_handle, data, total_length, 1000);
        slave_deselect();

        if(!response[0]){
            goto error;
        }
#if(SD_CARD_DEBUG)
        printf("SD card: opened file and got info\n");
#endif
        return 1;
    }else{
        return 0;
    }

    error:
#if(SD_CARD_DEBUG)
        printf("SD card: failed to opened file\n");
#endif
        slave_deselect();
        return 0;
}

uint8_t sd_write_data_to_file(const char *data){
    if(m_device_handle){
        uint8_t response[1];
        uint8_t command = LOGGER_SD_WRITE_DATA_TO_FILE;

        uint16_t total_length = strlen(data) + 1; // string, \0
        uint8_t transmit_length[] = {(total_length >> 8) & 0xFF, total_length & 0xFF};
        uint8_t transmit_buffer[total_length]; // prob dont need to put this into a new array
        memcpy(&transmit_buffer[0], data, total_length); // include string terminator

        HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000); // send command
        if(!wait_for_slave_ready(1000)) goto error;
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, transmit_length, 2, 5000); // send how many bytes the dat will be
        if(!wait_for_slave_ready(1000)) goto error;
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, transmit_buffer, total_length, 5000); // send data
        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();

        if(!response[0]){
            goto error;
        }
#if(SD_CARD_DEBUG)
        printf("SD card: wrote data\n");
#endif
        return 1;
    }else{
        return 0;
    }

    error:
#if(SD_CARD_DEBUG)
        printf("SD card: failed to write data\n");
#endif
        slave_deselect();
        return 0;
};

uint8_t sd_read_data_from_file(){
    if(m_device_handle){
        uint8_t command = LOGGER_SD_READ_DATA_FORM_FILE;
        uint8_t response[1];

        HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();

#if(SD_CARD_DEBUG)
        printf("SD card: read data\n");
#endif
        return 1;
    }else{
        return 0;
    }

    error:
#if(SD_CARD_DEBUG)
        printf("SD card: failed to read data\n");
#endif
        slave_deselect();
        return 0;
}

uint8_t sd_set_file_cursor_offset(uint32_t cursor){
    if(m_device_handle){
        uint8_t response[1];
        uint8_t command = LOGGER_SD_SET_FILE_CURSOR_OFFSET;
        uint8_t transmit_buffer[] = {
            (cursor >> 24) & 0xFF, 
            (cursor >> 16) & 0xFF, 
            (cursor >> 8) & 0xFF,  
            cursor & 0xFF
        };

        HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000); // command
        if(!wait_for_slave_ready(1000)) goto error;
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, transmit_buffer, 4, 5000); // data
        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();
        if(!response[0]){
            goto error;
        }
#if(SD_CARD_DEBUG)
        printf("SD card: set cursor\n");
#endif
        return 1;
    }else{
        return 0;
    }

    error:
#if(SD_CARD_DEBUG)
        printf("SD card: failed to set cursor\n");
#endif
        slave_deselect();
        return 0;
}

uint8_t sd_close_file(){
    if(m_device_handle){
        uint8_t command = LOGGER_SD_CLOSE_FILE;
        uint8_t response[1];

        HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();

        if(!response[0]){
            goto error;
            return 0;
        }
#if(SD_CARD_DEBUG)
        printf("SD card: closed file\n");
#endif
        return 1;
    }else{
        return 0;
    }

    error:
#if(SD_CARD_DEBUG)
        printf("SD card: failed to close file\n");
#endif
        slave_deselect();
        return 0;
}

uint8_t sd_card_deinitialize(){
    if(m_device_handle){
        uint8_t command = LOGGER_SD_CARD_DEINITIALIZE;
        uint8_t response[1];

        HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();

        if(!response[0]){
            goto error;
            return 0;
        }
#if(SD_CARD_DEBUG)
        printf("SD card: deinitialize\n");
#endif
        return 1;
    }else{
        return 0;
    }

    error:
#if(SD_CARD_DEBUG)
        printf("SD card: failed to deinitialize\n");
#endif
        slave_deselect();
        return 0;
}

uint8_t sd_card_append_to_buffer(const char *string_format, ...){
    if(m_device_handle){
        uint8_t command = LOGGER_SD_CARD_APPEND_TO_BUFFER;
        uint8_t response[1];
        
        va_list args;
        va_start(args, string_format);

        uint16_t total_length = vsnprintf(
            string_buffer,
            SD_BUFFER_SIZE, 
            string_format, 
            args
        ) + 1;

        uint8_t transmit_length[] = {(total_length >> 8) & 0xFF, total_length & 0xFF};

        HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, transmit_length, 2, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, string_buffer, total_length, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();

        for(uint16_t i = 0; i < STRING_BUFFER_SIZE; i++){
            string_buffer[0] = 0;
        }

        if(!response[0]){
            goto error;
        }
#if(SD_CARD_DEBUG)
        printf("SD card: set cursor\n");
#endif
        return 1;
    }

    error:
#if(SD_CARD_DEBUG)
        printf("SD card: failed to set cursor\n");
#endif
        slave_deselect();
        return 0;
}

// Check for null results from this
char* sd_card_get_buffer_pointer(){
    if(m_device_handle){
        uint8_t command = LOGGER_SD_GET_BUFFER_POINTER;
        uint8_t receive_size_buffer[2];

        HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        start_waiting_for_slave_ready();
        status = HAL_SPI_Receive(m_device_handle, receive_size_buffer, 2, 5000);


        uint16_t buffer_size = receive_size_buffer[0] << 8 | receive_size_buffer[1];
        if(buffer_size == 0 ) goto error; // not error but error response is the correct one

        char* receive_buffer = (char*)malloc(buffer_size);
        receive_buffer[0] = '\0'; // Terminate just in case

        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, (uint8_t*)receive_buffer, buffer_size, 5000);
        slave_deselect();

        return receive_buffer;
    }else{
        return NULL;
    }

    error:
#if(SD_CARD_DEBUG)
        printf("SD card: failed to get buffer\n");
#endif
        slave_deselect();
        return NULL;
}

uint32_t sd_card_get_selected_file_size(){
    if(m_device_handle){
        uint8_t command = LOGGER_SD_GET_SELECTED_FILE_SIZE;
        uint8_t response[4];

        HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 4, 5000);
        slave_deselect();

        uint32_t file_size = response[0] << 24 | response[1] << 16 | response[2] << 8 | response[3];
        return file_size;
    }else{
        return 0;
    }

    error:
#if(SD_CARD_DEBUG)
        printf("SD card: failed to get selected file size\n");
#endif
        slave_deselect();
        return 0;
}

uint8_t sd_write_buffer_to_file(){

    if(m_device_handle){
        uint8_t command = LOGGER_SD_WRITE_BUFFER_TO_FILE;
        uint8_t response[1];

        HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();

        if(!response[0]){
            goto error;
        }
#if(SD_CARD_DEBUG)
        printf("SD card: wrote buffer data\n");
#endif
        return 1;
    }else{
        return 0;
    }

    error:
#if(SD_CARD_DEBUG)
        printf("SD card: failed to write buffer data\n");
#endif
        slave_deselect();
        return 0;
}

uint8_t sd_file_exists(const char *file_name){
    if(m_device_handle){
        uint8_t response[1];
        uint8_t command = LOGGER_SD_FILE_EXISTS;
        uint16_t total_length = strlen(file_name)+1; // string length, \0
        uint8_t transmit_length[] = {(total_length >> 8) & 0xFF, total_length & 0xFF};
        uint8_t transmit_buffer[total_length]; 

        memcpy(transmit_buffer, file_name, total_length); // Copy the string including null terminator

        HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, transmit_length, 2, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, transmit_buffer, total_length, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();

        if(!response[0]){
            goto error;
            return 0;
        }
#if(SD_CARD_DEBUG)
        printf("SD card: File exists\n");
#endif
        return 1;
    }else{
        return 0;
    }

    error:
#if(SD_CARD_DEBUG)
        printf("SD card: File does not exist\n");
#endif
        slave_deselect();
        return 0;
}

uint8_t sd_save_file(){
    if(m_device_handle){
        uint8_t command = LOGGER_SD_SAVE_FILE;
        uint8_t response[1];

        HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();
        if(!response[0]){
            goto error;
            return 0;
        }
#if(SD_CARD_DEBUG)
        printf("SD card: File saved\n");
#endif
        return 1;
    }else{
        return 0;
    }

    error:
#if(SD_CARD_DEBUG)
        printf("SD card: File was not saved\n");
#endif
        slave_deselect();
        return 0;
}

uint8_t sd_test_interface(){
    if(m_device_handle){
        uint8_t command = LOGGER_TEST_INTERFACE;
        uint8_t response[1];
        volatile HAL_StatusTypeDef status;
        slave_select();
        start_waiting_for_slave_ready();
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 5000);
        if(!wait_for_slave_ready(1000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();

        if(response[0] == LOGGER_INTERFACE_TEST_VALUE){
            return 1;
        }else{
            return 0;
        }
    }else{
        return 1;
    }

    error:
        slave_deselect();
        return 0;
}

// Example code ************************************************
    // sd_card_initialize();
    // sd_open_file("Quadcopter.txt", FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
    // sd_set_file_cursor_offset(sd_card_get_selected_file_size());
    // sd_write_data_to_file("Quadcopter restarted\n");
    // sd_write_data_to_file("Quadcopter data2\n");
    // sd_write_data_to_file("Quadcopter data3\n");
    // sd_write_data_to_file("Quadcopter data4\n");
    // sd_close_file();
    // sd_open_file("Quadcopter.txt", FA_READ);
    // sd_read_data_from_file();
    // printf("Read Data : %s\n", sd_card_get_buffer_pointer());
    // sd_buffer_clear();
    // sd_close_file();
    // sd_open_file("Quadcopter.txt", FA_WRITE);
    // sd_set_file_cursor_offset(sd_card_get_selected_file_size()); // set cursor to end of file to so data is not overwritten
    // sd_write_data_to_file("Quadcopter restarted\n");
    // sd_write_data_to_file("Quadcopter data2\n");
    // sd_write_data_to_file("Quadcopter data3\n");
    // sd_write_data_to_file("Quadcopter data4\n");
    // sd_close_file();
    // sd_open_file("Quadcopter.txt", FA_READ);
    // sd_read_data_from_file();
    // printf("Read Data : %s\n", sd_card_get_buffer_pointer());
    // sd_buffer_clear();
    // sd_close_file();
    // sd_card_deinitialize();