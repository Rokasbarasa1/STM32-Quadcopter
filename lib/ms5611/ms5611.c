#include "ms5611.h"

#define MS5661_I2C_ID 0x76

enum t_ms5611_commands {
    MS5611_RESET            = 0b00011110,
    MS5611_D1_OSR_256       = 0b01000000,
    MS5611_D1_OSR_512       = 0b01000010,
    MS5611_D1_OSR_1024      = 0b01000100,
    MS5611_D1_OSR_2048      = 0b01000110,
    MS5611_D1_OSR_4096      = 0b01001000,
    MS5611_D2_OSR_256       = 0b01010000,
    MS5611_D2_OSR_512       = 0b01010010,
    MS5611_D2_OSR_1024      = 0b01010100,
    MS5611_D2_OSR_2048      = 0b01010110,
    MS5611_D2_OSR_4096      = 0b01011000,
    MS5611_ADC_READ         = 0b00000000,
    MS5611_PROM_READ        = 0b10100000,

    MS5611_SET_PROM_MODE    = 0b10100110,
};

uint16_t pressure_sensitivity = 0;
uint16_t pressure_offset = 0;
uint16_t temp_coefficient_of_pressure_sensitivity = 0;
uint16_t temp_coefficient_of_pressure_offset = 0;
uint16_t reference_temperature = 0;
uint16_t temperature_coefficient_of_the_temperature = 0;

uint32_t digital_pressure_value = 0;
uint32_t digital_temperature_value = 0;

int32_t actual_temperature = 0;

I2C_HandleTypeDef *i2c_handle;

uint8_t init_ms5611(I2C_HandleTypeDef *i2c_handle_temp){
    i2c_handle = i2c_handle_temp;

    // ################# Reset it
    HAL_StatusTypeDef ret1 = HAL_I2C_Mem_Write(
        i2c_handle, 
        MS5661_I2C_ID, 
        MS5611_RESET, 
        1, 
        NULL, 
        0, 
        1000
    );

    uint8_t reset_cmd = MS5611_RESET;
    HAL_StatusTypeDef ret_new = HAL_I2C_Master_Transmit(
        i2c_handle, 
        MS5661_I2C_ID << 1, // Shift the address to the left to add the write bit
        &reset_cmd, 
        1, 
        1000
    );

    if (HAL_I2C_IsDeviceReady(i2c_handle, MS5661_I2C_ID << 1, 10, 1000) != HAL_OK) {
        printf("MS5611: Device is not ready\n");
    } else {
        printf("MS5611: Device is ready\n");
    }

    if (ret_new != HAL_OK) {
        printf("MS5611: Error reseting new \n");
        // return 0;
    }

    if (ret1 != HAL_OK) {
        printf("MS5611: Error reseting old \n");
        // return 0;
    }

    // ############## Read PROM
    // Set PROM mode
    HAL_StatusTypeDef ret2 = HAL_I2C_Mem_Write(
        i2c_handle, 
        MS5661_I2C_ID, 
        MS5611_SET_PROM_MODE, 
        1, 
        NULL, 
        0, 
        100
    );

    if (ret2 != HAL_OK) {
        printf("MS5611: Error setting PROM mode\n");
        return 0;
    }

    // Read the actual PROM data
    // uint8_t prom_data[16] = {0}; // 16 Because the total PROM size is 128 bits

    uint8_t prom_data[2] = {0}; // Buffer to hold one 16-bit word of PROM data at a time

    // Read each PROM coefficient
    for (int i = 0; i < 8; i++) {
        uint8_t command = 0xA0 | (i << 1); // Replace with the correct command for each coefficient

        HAL_StatusTypeDef ret3 = HAL_I2C_Mem_Read(
            i2c_handle,
            MS5661_I2C_ID,
            command, // Use the command as "memory address"
            1,
            prom_data,
            2, // Read two bytes (16 bits) at a time
            100
        );

        if (ret3 != HAL_OK) {
            // Handle error
        }

        // Process the received PROM data
        printf("PROM coefficient %d: %d\n", i, (prom_data[0] << 8) | prom_data[1]);
    }



    // HAL_StatusTypeDef ret3 = HAL_I2C_Mem_Read(
    //     i2c_handle,
    //     MS5661_I2C_ID,
    //     NULL,
    //     0,
    //     prom_data,
    //     2,
    //     100
    // );

    // printf("After loading\n");
    // for(uint8_t i = 0; i < 16; i++){
    //     printf("%d ", prom_data[i]);
    // }
    // printf("\n");

    // Check crc
    printf("MS5611 initialized\n");

    return 1;
}
