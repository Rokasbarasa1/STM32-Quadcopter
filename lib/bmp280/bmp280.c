#include "./bmp280.h"

#define BMP280_I2C_ID (0x76 << 1)
#define ID_REG 0xD0
#define ID_VALUE 0x58

#define TRIM_REG 0x88
#define RESET_REG 0xE0
#define RESET_VALUE 0xB6

#define CTRL_MEAS_REG 0xF4
#define CONFIG_REG 0xF5
#define PRES_MSB_REG 0xF7
#define TEMP_MSB_REG 0xFA

#define TEMP_LAPSE_RATE 0.0065
#define CELSIUS_TO_KELVIN 273.15
#define GRAVITY 9.80665
#define DRY_AIR_MOLAR_MASS 0.0289644
#define DRY_AIR_GAS_CONSTANT 287.058

enum t_temp_oversampling {
    OS_TEMP_1  = 0b00100000,
    OS_TEMP_2  = 0b01000000,
    OS_TEMP_4  = 0b01100000,
    OS_TEMP_8  = 0b10000000,
    OS_TEMP_16 = 0b10100000
};

enum t_pres_oversampling {
    OS_PRES_1  = 0b00000100,
    OS_PRES_2  = 0b00001000,
    OS_PRES_4  = 0b00001100,
    OS_PRES_8  = 0b00010000,
    OS_PRES_16 = 0b00010100
};

enum t_power_modes {
    SLEEP_MODE  = 0b00000000,
    FORCED_MODE = 0b00000001,
    NORMAL_MODE = 0b00000011,
};

enum t_standby_modes {
    SB_MODE_0_5  = 0b00000000,
    SB_MODE_62_5 = 0b00000001,
    SB_MODE_125  = 0b00000010,
    SB_MODE_250  = 0b00000011,
    SB_MODE_500  = 0b00000100,
    SB_MODE_1000 = 0b00000101,
    SB_MODE_2000 = 0b00000110,
    SB_MODE_4000 = 0b00000111,
};

enum t_filter_modes {
    FILTER_MODE_1  = 0b00000000,
    FILTER_MODE_2  = 0b00000100,
    FILTER_MODE_4  = 0b00001000,
    FILTER_MODE_8  = 0b00001100,
    FILTER_MODE_16 = 0b00010000,
};

I2C_HandleTypeDef *i2c_handle;

float reference_pressure = 0.0;

// For storing the trim values for pressure and temperature
uint16_t dig_T1,
    dig_P1;
int16_t dig_T2, dig_T3,
    dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

int32_t bmp280_convert_raw_temp(int32_t adc_T);
uint32_t bmp280_convert_raw_pres(int32_t adc_P);
uint8_t load_trim_registers();

uint8_t init_bmp280(I2C_HandleTypeDef *i2c_handle_temp)
{
    i2c_handle = i2c_handle_temp;

    // Check that the i2c device is there
    uint8_t check = 0;
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        i2c_handle, 
        BMP280_I2C_ID + 1, 
        ID_REG, 
        1, 
        &check, 
        1, 
        100);

    // Check if reading was ok and if the value that was read is correct, matches id value
    if (ret != HAL_OK || check != ID_VALUE)
    {
        printf("BMP280 initialization failed\n");
        return 0;
    }

    uint8_t reset_device = 0b00000000;
    reset_device |= RESET_VALUE; // A specific reset value has to be applied to the register

    HAL_StatusTypeDef ret1 = HAL_I2C_Mem_Write(
        i2c_handle, 
        BMP280_I2C_ID, 
        RESET_REG, 
        1, 
        &reset_device, 
        1, 100);

    uint8_t ctrl_meas_register = 0b00000000;
    ctrl_meas_register |= NORMAL_MODE; // set it to normal mode
    ctrl_meas_register |= OS_TEMP_1; // enable temp measurement oversampling x1
    ctrl_meas_register |= OS_PRES_16; // set pressure oversampling to standard resolution 18bit/0.66 PA x4
    HAL_StatusTypeDef ret2 = HAL_I2C_Mem_Write(
        i2c_handle, 
        BMP280_I2C_ID, 
        CTRL_MEAS_REG, 
        1, 
        &ctrl_meas_register, 
        1, 
        100);

    uint8_t config_register = 0b00000000;
    config_register |= SB_MODE_0_5; // Set standby time to 0.5mx, disable spi on last bit
    config_register |= FILTER_MODE_16; // Set filter to coefficient 2
    HAL_StatusTypeDef ret3 = HAL_I2C_Mem_Write(
        i2c_handle, 
        BMP280_I2C_ID, 
        CONFIG_REG, 
        1, 
        &config_register, 
        1, 
        100);

    printf("BMP280 initialized\n");
    HAL_Delay(100); // Wait a bit to let the new settings soak in
    uint8_t load_trims = load_trim_registers();

    return ret1 == HAL_OK && ret2 == HAL_OK && ret3 == HAL_OK && load_trims == 1;
}

// Use the stored trim data to get the value for temperature
int32_t bmp280_convert_raw_temp(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >>
           14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Use the stored trim data to get the value for pressure
uint32_t bmp280_convert_raw_pres(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)p;
}

// Load the trim data that is hardcoded into the bmp280 and store it in the variables.
uint8_t load_trim_registers()
{
    uint8_t trim_data[26];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        i2c_handle,
        BMP280_I2C_ID + 1,
        TRIM_REG,
        1,
        trim_data,
        26, // there are 26 bits of trim data
        100);

    // Check if reading failed
    if (ret != HAL_OK)
    {
        return 0;
    }

    dig_T1 = (trim_data[1] << 8) | trim_data[0];
    dig_T2 = (trim_data[3] << 8) | trim_data[2];
    dig_T3 = (trim_data[5] << 8) | trim_data[4];
    dig_P1 = (trim_data[7] << 8) | trim_data[5];
    dig_P2 = (trim_data[9] << 8) | trim_data[6];
    dig_P3 = (trim_data[11] << 8) | trim_data[10];
    dig_P4 = (trim_data[13] << 8) | trim_data[12];
    dig_P5 = (trim_data[15] << 8) | trim_data[14];
    dig_P6 = (trim_data[17] << 8) | trim_data[16];
    dig_P7 = (trim_data[19] << 8) | trim_data[18];
    dig_P8 = (trim_data[21] << 8) | trim_data[20];
    dig_P9 = (trim_data[23] << 8) | trim_data[22];

    return 1;
}

float bmp280_get_pressure_hPa()
{
    // For storing the bytes of temperature data that is going to be read
    uint8_t retrieved_data[] = {0, 0, 0};

    //  Registers will be read in this order
    //  press_msb
    //  press_lsb
    //  press_xlsb
    HAL_I2C_Mem_Read(
        i2c_handle,
        BMP280_I2C_ID + 1,
        PRES_MSB_REG,
        1,
        retrieved_data,
        3,
        100);

    // Combine the 3 bytes in a specific way to get a 16-20 bit value
    int32_t combined_pres = ((int32_t)retrieved_data[0] << 12) | ((int32_t)retrieved_data[1]) << 4 | ((int32_t)retrieved_data[2] >> 4);
    float pressure = ((float)bmp280_convert_raw_pres(combined_pres)) / 256.0;

    // Return the value after dividing by 100 to convert Pa -> hPa
    return pressure / 100;
}

float bmp280_get_temperature_celsius()
{
    // For storing the bytes of temperature data that is going to be read
    uint8_t retrieved_data[] = {0, 0, 0};

    //  Registers will be read in this order
    //  temp_msb
    //  temp_lsb
    //  temp_xlsb
    HAL_I2C_Mem_Read(
        i2c_handle,
        BMP280_I2C_ID + 1,
        TEMP_MSB_REG,
        1,
        retrieved_data,
        3,
        100);

    // Combine the 3 bytes in a specific way to get a 16-20 bit value
    int32_t combined_temp = ((int32_t)retrieved_data[0] << 12) | ((int32_t)retrieved_data[1]) << 4 | ((int32_t)retrieved_data[2] >> 4);
    float temperature = ((float)bmp280_convert_raw_temp(combined_temp)) / 100.0;
    return temperature;
}

float bmp280_get_height_meters_above_sea_level(float pressure_sea_level_hpa, float temperature_sea_level){
    float pressure = bmp280_get_pressure_hPa();

    return 44330 * (1.0 - pow(pressure / pressure_sea_level_hpa, 0.1903));
}

float bmp280_get_height_meters_from_reference(uint8_t reset_reference){
    float pressure = bmp280_get_pressure_hPa();

    if(reference_pressure == 0.0 || reset_reference == 1){
        // printf("data: %.2f %.2f\n",reference_pressure, pressure);
        reference_pressure = pressure;
    }
    
    // printf("44330 * (1.0 - pow(%.2f / %.2f, 0.1903))\n", pressure, reference_pressure);
    return 44330 * (1.0 - pow(pressure / reference_pressure, 0.1903));
}