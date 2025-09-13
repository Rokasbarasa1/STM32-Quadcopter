#include "./sd_card_spi.h"

#define SD_CARD_DEBUG 1

uint8_t driver_initialized = 0;

SPI_HandleTypeDef * m_device_handle;
GPIO_TypeDef* m_slave_select_port = GPIOB;
uint16_t m_slave_select_pin = GPIO_PIN_11;

GPIO_TypeDef* m_slave_ready_port = GPIOB;
uint16_t m_slave_ready_pin = GPIO_PIN_11;

uint8_t sd_buffer0[SD_BUFFER_SIZE];
uint8_t sd_buffer1[SD_BUFFER_SIZE];
uint16_t sd_buffer0_index = 0;
uint16_t sd_buffer1_index = 0;

uint8_t selected_buffer = 0;

#define STRING_BUFFER_SIZE 300
uint8_t string_buffer[STRING_BUFFER_SIZE];

volatile uint8_t slave_ready = 0;


volatile uint8_t dma_transfer_call_status = 0;

volatile uint8_t response_code = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
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

uint16_t sd_buffer_size(uint8_t local){
    if(local){
        if(selected_buffer == 0) return sd_buffer0_index - 1;
        if(selected_buffer == 1) return sd_buffer1_index - 1;
    }

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

uint8_t sd_buffer_clear_index(uint8_t local){
    if(local){
        if(selected_buffer == 0){
            sd_buffer0_index = 0;
            return 1;
        }else if(selected_buffer == 1){
            sd_buffer1_index = 0;
            return 1;
        }
    }

    // Non local one is to be implemented
}

uint8_t sd_buffer_clear(uint8_t local){
    if(local){
        if(selected_buffer == 0){
            memset(sd_buffer0, 0, SD_BUFFER_SIZE * sizeof(uint8_t));
            // for(int i=0; i< SD_BUFFER_SIZE; i++){
            //     sd_buffer0[i] = 0;
            // }
            sd_buffer0_index = 0;
            return 1;
        }else if(selected_buffer == 1){
            memset(sd_buffer1, 0, SD_BUFFER_SIZE * sizeof(uint8_t));
            // for(int i=0; i< SD_BUFFER_SIZE; i++){
            //     sd_buffer1[i] = 0;
            // }
            sd_buffer1_index = 0;
            return 1;
        }
    }

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

uint8_t sd_card_append_to_buffer(uint8_t local, const char *string_format, ...){
    if(local){
        if(selected_buffer == 0){
            va_list args;
            va_start(args, string_format);
            // Ensure we don't write beyond the buffer
            int written = vsnprintf(&sd_buffer0[sd_buffer0_index], SD_BUFFER_SIZE - sd_buffer0_index, string_format, args);
            if (written > 0) {
                sd_buffer0_index += written < (SD_BUFFER_SIZE - sd_buffer0_index) ? written : (SD_BUFFER_SIZE - sd_buffer0_index - 1);
            }
            va_end(args);
            return 1;
        }else if(selected_buffer == 1){
            va_list args;
            va_start(args, string_format);
            // Ensure we don't write beyond the buffer
            int written = vsnprintf(&sd_buffer1[sd_buffer1_index], SD_BUFFER_SIZE - sd_buffer1_index, string_format, args);
            if (written > 0) {
                sd_buffer1_index += written < (SD_BUFFER_SIZE - sd_buffer1_index) ? written : (SD_BUFFER_SIZE - sd_buffer1_index - 1);
            }
            va_end(args);
            return 1;
        }
    }

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
char* sd_card_get_buffer_pointer(uint8_t local){
    if(local){
        if(selected_buffer == 0) return sd_buffer0;
        else if(selected_buffer == 1) return sd_buffer1;
    }

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

void sd_card_buffer_increment_index(){
    if(selected_buffer == 0) sd_buffer0_index++;
    else if(selected_buffer == 1) sd_buffer1_index++;
}

void sd_card_buffer_increment_index_by_amount(uint16_t index_increment_amount){
    if(selected_buffer == 0) sd_buffer0_index += index_increment_amount;
    else if(selected_buffer == 1) sd_buffer1_index += index_increment_amount;
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
    // if(local){
        // i dont care now
        // return 0;
    // }

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
        slave_select();
        start_waiting_for_slave_ready();
        volatile HAL_StatusTypeDef status;
        status = HAL_SPI_Transmit(m_device_handle, &command, 1, 60000);
        if(!wait_for_slave_ready(60000)) goto error;
        status = HAL_SPI_Receive(m_device_handle, response, 1, 60000);
        slave_deselect();

        if(response[0] == LOGGER_INTERFACE_TEST_VALUE){
            return 1;
        }else{
            return 0;
        }
    }else{
        return 0;
    }

    error:
        slave_deselect();
        return 0;
}


uint8_t sd_special_initialize(const char *file_base_name){
    if(m_device_handle){
        uint8_t response[1];
        uint8_t command = LOGGER_INITIALIZE;

        uint16_t file_length = strlen(file_base_name)+1; // \0 character as well
        uint16_t total_length = file_length;

        uint8_t transmit_length[] = {(total_length >> 8) & 0xFF, total_length & 0xFF};

        slave_select();
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, &command, 1, 60000); // send command
        if(!wait_for_slave_ready(60000)) goto error;
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, transmit_length, 2, 60000); // send how many bytes the dat will be
        if(!wait_for_slave_ready(60000)) goto error;
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, file_base_name, total_length, 60000); // send how many bytes the dat will be
        if(!wait_for_slave_ready(60000)) goto error;
        HAL_SPI_Receive(m_device_handle, response, 1, 60000);
        slave_deselect();

        response_code = response[0];
        if(response[0] != 0){ // For this case this is the actual status code from the fatfs driver.
            goto error;
        }

        return 1;
    }else{
        return 0;
    }

    error:
        slave_deselect();
        return 0;
}

uint16_t sd_special_get_file_index(){
    if(m_device_handle){
        uint8_t response[2];
        uint8_t command = LOGGER_GET_FILE_INDEX;

        slave_select();
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, &command, 1, 60000); // send command
        if(!wait_for_slave_ready(60000)) goto error;
        HAL_SPI_Receive(m_device_handle, response, 2, 60000);
        slave_deselect();

        uint16_t file_index = response[0] << 8 | response[1];
        if(response[0] != 0){ // For this case this is the actual status code from the fatfs driver.
            goto error;
        }

        printf("GOT index: %d (%d %d)\n", file_index, response[0], response[1]);
        return file_index;
    }else{
        return 0;
    }

    error:
        slave_deselect();
        return 0;
}

uint8_t sd_special_reset(){
    if(m_device_handle){
        uint8_t response[1];
        uint8_t command = LOGGER_RESET;

        slave_select();
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, &command, 1, 5000); // send command
        if(!wait_for_slave_ready(5000)) goto error;
        HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();
        
        if(!response[0]){
            goto error;
        }

        return 1;
    }else{
        return 0;
    }

    error:
        slave_deselect();
        return 0;
}

uint8_t sd_special_write_chunk_of_string_data(const char *data){
    if(m_device_handle){
        uint8_t response[1];
        uint8_t command = LOGGER_WRITE_CHUNK_OF_STRING_DATA;

        uint16_t file_length = strlen(data)+1; // \0 character as well
        uint16_t total_length = file_length;

        uint8_t transmit_length[] = {(total_length >> 8) & 0xFF, total_length & 0xFF};

        slave_select();
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, &command, 1, 5000); // send command
        if(!wait_for_slave_ready(1000)) goto error;
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, transmit_length, 2, 5000); // send how many bytes the dat will be
        if(!wait_for_slave_ready(1000)) goto error;
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, data, total_length, 5000); // send how many bytes the dat will be
        if(!wait_for_slave_ready(1000)) goto error;
        HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();

        if(!response[0]){
            goto error;
        }

        return 1;
    }else{
        return 0;
    }

    error:
        slave_deselect();
        return 0;
}


uint8_t sd_special_write_chunk_of_byte_data(const char *data, uint16_t length){
    if(m_device_handle){
        uint8_t response[1];
        uint8_t command = LOGGER_WRITE_CHUNK_OF_BYTE_DATA;

        uint8_t transmit_length[] = {(length >> 8) & 0xFF, length & 0xFF};

        slave_select();
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, &command, 1, 5000); // send command
        if(!wait_for_slave_ready(1000)) goto error;
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, transmit_length, 2, 5000); // send how many bytes the dat will be
        if(!wait_for_slave_ready(1000)) goto error;
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, (uint8_t*)data, length, 5000); // send how many bytes the dat will be
        if(!wait_for_slave_ready(1000)) goto error;
        HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();

        if(!response[0]){
            goto error;
        }

        return 1;
    }else{
        return 0;
    }

    error:
        slave_deselect();
        return 0;
}

uint8_t sd_special_enter_async_string_mode(uint8_t reset_sd_initialization_when_async_stops){
    if(m_device_handle){
        uint8_t response[1];
        uint8_t command = LOGGER_ENTER_ASYNC_STRING_MODE;

        slave_select();
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, &command, 1, 60000); // send command
        if(!wait_for_slave_ready(60000)) goto error;
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, &reset_sd_initialization_when_async_stops, 1, 60000); // tell slave what to do when async is stopped by master
        if(!wait_for_slave_ready(60000)) goto error;
        HAL_SPI_Receive(m_device_handle, response, 1, 60000);
        slave_deselect();
        
        if(!response[0]){
            goto error;
        }

        return 1;
    }else{
        return 0;
    }

    error:
        slave_deselect();
        return 0;
}

uint8_t sd_special_enter_async_byte_mode(uint8_t reset_sd_initialization_when_async_stops){
    if(m_device_handle){
        uint8_t response[1];
        uint8_t command = LOGGER_ENTER_ASYNC_BYTE_MODE;

        slave_select();
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, &command, 1, 60000); // send command
        if(!wait_for_slave_ready(60000)) goto error;
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, &reset_sd_initialization_when_async_stops, 1, 60000); // tell slave what to do when async is stopped by master
        if(!wait_for_slave_ready(60000)) goto error;
        HAL_SPI_Receive(m_device_handle, response, 1, 60000);
        slave_deselect();
        
        if(!response[0]){
            goto error;
        }

        return 1;
    }else{
        return 0;
    }

    error:
        slave_deselect();
        return 0;
}

uint8_t sd_special_leave_async_mode(){
    if(m_device_handle){
        uint8_t response[1];
        uint8_t command = LOGGER_LEAVE_ASYNC_MODE;

        uint8_t command_array[250];
        for (size_t i = 0; i < 250; i++){
            command_array[i] = command;
        }
        

        slave_select();
        start_waiting_for_slave_ready();
        HAL_SPI_Transmit(m_device_handle, command_array, 250, 5000); // send command
        if(!wait_for_slave_ready(500)) goto error;
        HAL_SPI_Receive(m_device_handle, response, 1, 5000);
        slave_deselect();
        
        if(!response[0]){
            goto error;
        }

        return 1;
    }else{
        return 0;
    }

    error:
        slave_deselect();
        return 0;
}

uint8_t sd_special_write_chunk_of_string_data_no_slave_response(const char *data){
    if(m_device_handle){

        uint16_t file_length = strlen(data)+1; // \0 character as well
        uint16_t total_length = file_length;

        slave_select();
        HAL_SPI_Transmit(m_device_handle, data, total_length, 5000); // send how many bytes the dat will be
        slave_deselect();

        return 1;
    }else{
        return 0;
    }
}

uint8_t sd_special_write_chunk_of_byte_data_no_slave_response(const char *data, uint16_t length){
    if(m_device_handle){

        slave_select();
        HAL_SPI_Transmit(m_device_handle, data, length, 5000); // send how many bytes the dat will be
        slave_deselect();

        return 1;
    }else{
        return 0;
    }
}

void delay_us(uint32_t microseconds) {
    volatile uint32_t count = 0;
    const uint32_t delayLoops = (100 * microseconds);  // Adjust multiplier based on clock and loop execution time

    for(count = 0; count < delayLoops; count++) {
        // empty loop body
    }
}

uint8_t sd_special_write_chunk_of_string_data_async(const char *data){
    if(m_device_handle){

        // \0 character not needed anymore as the SD card interface now accepts non \0 terminated strings (arrays of uint8)
        uint16_t file_length = strlen(data);
        uint16_t total_length = file_length;

        uint8_t wait_status = sd_special_wait_until_async_write_done(); // PROBLEM
        if(wait_status == 0) return 0;

        volatile uint32_t wait_amount_max = 200000;
        volatile uint32_t wait_amount = 0;
        while (dma_transfer_call_status == 1 && wait_amount_max > wait_amount) wait_amount++;
        if(wait_amount_max < wait_amount){
            printf("Stopped waiting for dma_transfer_call_status as too many iterations\n");
            return 0;
        }
        
        slave_select(); 

        // Use DMA to not block 
        HAL_StatusTypeDef ret = HAL_SPI_Transmit_DMA(m_device_handle, (uint8_t*)data, total_length);

        if(ret != HAL_OK){
            printf("sd_card_spi: sd_special_write_chunk_of_string_data_async error %d\n", ret);
            return 0;
        }
        dma_transfer_call_status = 1;
        // Do not deselect the slave as it needs to be active in the background

        return 1;
    }else{
        return 0;
    }
}

uint8_t sd_special_write_chunk_of_byte_data_async(const char *data, uint16_t length){
    if(m_device_handle){

        // Wait until DMA transfer complete if there was a previous call
        uint8_t wait_status = sd_special_wait_until_async_write_done(); // PROBLEM
        if(wait_status == 0)return 0;
        while (dma_transfer_call_status == 1);
        slave_select(); 

        // Use DMA to not block 
        HAL_StatusTypeDef ret = HAL_SPI_Transmit_DMA(m_device_handle, (uint8_t*)data, length);

        if(ret != HAL_OK){
            printf("sd_card_spi: sd_special_write_chunk_of_byte_data_async error %d\n", ret);
            return 0;
        }
        dma_transfer_call_status = 1;
        // Do not deselect the slave as it needs to be active in the background

        return 1;
    }else{
        return 0;
    }
}

// 100 000 000

    //   9 073

uint8_t sd_special_wait_until_async_write_done(){
    uint32_t wait_count_max = 200000;
    uint32_t wait_count = 0;
    while (HAL_SPI_GetState(m_device_handle) != HAL_SPI_STATE_READY && wait_count > wait_count_max){
        wait_count++;
    };

    if(wait_count > wait_count_max){
        printf("Stopped waiting. Count over limit\n");
        return 0;
    }
    return 1;
}

void sd_buffer_swap(){
    if(selected_buffer == 0){
        selected_buffer = 1;
    }else if(selected_buffer == 1){
        selected_buffer = 0;
    }
}

uint8_t sd_get_response(){
    
	// FR_OK = 0,				/* (0) Succeeded */
	// FR_DISK_ERR,			    /* (1) A hard error occurred in the low level disk I/O layer */
	// FR_INT_ERR,				/* (2) Assertion failed */
	// FR_NOT_READY,			/* (3) The physical drive cannot work */
	// FR_NO_FILE,				/* (4) Could not find the file */
	// FR_NO_PATH,				/* (5) Could not find the path */
	// FR_INVALID_NAME,		    /* (6) The path name format is invalid */
	// FR_DENIED,				/* (7) Access denied due to prohibited access or directory full */
	// FR_EXIST,				/* (8) Access denied due to prohibited access */
	// FR_INVALID_OBJECT,		/* (9) The file/directory object is invalid */
	// FR_WRITE_PROTECTED,		/* (10) The physical drive is write protected */
	// FR_INVALID_DRIVE,		/* (11) The logical drive number is invalid */
	// FR_NOT_ENABLED,			/* (12) The volume has no work area */
	// FR_NO_FILESYSTEM,		/* (13) There is no valid FAT volume */
	// FR_MKFS_ABORTED,		    /* (14) The f_mkfs() aborted due to any problem */
	// FR_TIMEOUT,				/* (15) Could not get a grant to access the volume within defined period */
	// FR_LOCKED,				/* (16) The operation is rejected according to the file sharing policy */
	// FR_NOT_ENOUGH_CORE,		/* (17) LFN working buffer could not be allocated */
	// FR_TOO_MANY_OPEN_FILES,	/* (18) Number of open files > _FS_LOCK */
	// FR_INVALID_PARAMETER 	/* (19) Given parameter is invalid */

    return response_code;
}

void sd_card_set_dma_transfer_call_status(uint8_t status){
    dma_transfer_call_status = status;
    if(dma_transfer_call_status == 0){
        slave_deselect();
    }
}

void sd_card_wait_for_dma_transfer_complete(){
    while(dma_transfer_call_status);
    return;
}

void sd_card_wait_for_slave_ready(){
    wait_for_slave_ready(5000);
}

// Example code ************************************************
    // uint8_t sd_status = 0;
    // sd_status = sd_card_initialize_spi(&hspi3, GPIOA, GPIO_PIN_15, GPIOA, GPIO_PIN_12);
    // if(!sd_test_interface()){
    //     printf("FAILED SD INTERFACE TEST\n");
    //     while (1);
    // }
    // sd_status = sd_card_initialize();
    // sd_status = sd_open_file("Quadcopter.txt", FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
    // sd_status = sd_close_file();
    // sd_status = sd_card_deinitialize();

    // sd_status = sd_card_initialize();
    // sd_status = sd_open_file("Quadcopter.txt", FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
    // sd_status = sd_set_file_cursor_offset(sd_card_get_selected_file_size());
    // sd_status = sd_write_data_to_file("Quadcopter restarted\n");
    // sd_status = sd_write_data_to_file("Quadcopter data2\n");
    // sd_status = sd_write_data_to_file("Quadcopter data3\n");
    // sd_status = sd_write_data_to_file("Quadcopter data4\n");
    // sd_status = sd_close_file();
    // sd_status = sd_open_file("Quadcopter.txt", FA_READ);
    // sd_status = sd_read_data_from_file();
    // char* data = sd_card_get_buffer_pointer(0);
    // if(data){
    //     sd_status = printf("Read Data : %s\n", data);
    // }
    // sd_buffer_clear(0);
    // sd_status = sd_close_file();
    // sd_status = sd_open_file("Quadcopter.txt", FA_WRITE);
    // sd_status = sd_set_file_cursor_offset(sd_card_get_selected_file_size()); // set cursor to end of file to so data is not overwritten
    // sd_status = sd_write_data_to_file("Quadcopter restarted\n");
    // sd_status = sd_write_data_to_file("Quadcopter data2\n");
    // sd_status = sd_write_data_to_file("Quadcopter data3\n");
    // sd_status = sd_write_data_to_file("Quadcopter data4\n");
    // sd_status = sd_close_file();
    // sd_status = sd_open_file("Quadcopter.txt", FA_READ);
    // sd_status = sd_read_data_from_file();
    // char* data1 = sd_card_get_buffer_pointer(0);
    // if(data1){
    //     sd_status = printf("Read Data : %s\n", data1);
    //     free(data1);
    // }

    // sd_buffer_clear(0);
    // sd_status = sd_close_file();
    // sd_status = sd_card_deinitialize();