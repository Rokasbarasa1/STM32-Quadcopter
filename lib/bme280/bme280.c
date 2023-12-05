#include "./bmp280.h"

#define BME280_I2C_ID (0x76 << 1)

#define ID_VALUE 0x60

enum t_bmp280_registers {
    REGISTER_HUM_LSB             = 0xFE,
    REGISTER_HUM_MSB             = 0xFD,
    REGISTER_TEMP_XLSB           = 0xFC,
    REGISTER_TEMP_LSB            = 0xFB,
    REGISTER_TEMP_MSB            = 0xFA,
    REGISTER_PRESS_XLSB          = 0xF9,
    REGISTER_PRESS_LSB           = 0xF8,
    REGISTER_PRESS_MSB           = 0xF7,
    REGISTER_CONFIG              = 0xF5,
    REGISTER_CTRL_MEAS           = 0xF4,
    REGISTER_STATUS              = 0xF3,
    REGISTER_CTRL_HUM            = 0xF2,
    REGISTER_CALIB_HIGH_START    = 0xE1,
    REGISTER_CALIB_HIGH_END      = 0xF0,
    REGISTER_RESET               = 0xE0,
    REGISTER_ID                  = 0xD0,
    REGISTER_CALIB_LOW_START     = 0x88,
    REGISTER_CALIB_LOW_END       = 0xA1,
};

#define RESET_VALUE 0xB6

#define TEMP_LAPSE_RATE 0.0065
#define CELSIUS_TO_KELVIN 273.15
#define GRAVITY 9.80665
#define DRY_AIR_MOLAR_MASS 0.0289644
#define DRY_AIR_GAS_CONSTANT 287.058

I2C_HandleTypeDef *i2c_handle;

uint8_t init_bme280(I2C_HandleTypeDef *i2c_handle_temp){
    i2c_handle = i2c_handle_temp;

    // #################### Test the id of the sensor 
    uint8_t check = 0;
    HAL_StatusTypeDef res = HAL_I2C_Mem_Read(
        i2c_handle, 
        BME280_I2C_ID, 
        REGISTER_ID, 
        1, 
        &check, 
        1, 
        100
    );

    if (res != HAL_OK || check != ID_VALUE)
    {
        printf("BME280: Error getting id\n");
        return 0;
    }
    
    // #################### Reset the sensor
    uint8_t reset_device = RESET_VALUE;
    HAL_StatusTypeDef res1 = HAL_I2C_Mem_Write(
        i2c_handle, 
        BME280_I2C_ID, 
        REGISTER_RESET, 
        1, 
        &reset_device, 
        1, 
        100
    );

    if (res1 != HAL_OK) {
        printf("BME280: Error reseting\n");
        return 0;
    }

    printf("BME280 initialized\n");
    return 1;
}