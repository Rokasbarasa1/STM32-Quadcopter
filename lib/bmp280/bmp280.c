#include "./bmp280.h"


#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 


#define BMP280 (0x76 << 1)

// Storing values and functions that handle the conversion of the read data

// Trims are hardcoded into the bmp280
uint16_t dig_T1,  \
         dig_P1;

int16_t  dig_T2, dig_T3, \
         dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

// Copied from the datasheet
int32_t t_fine;
int32_t bmp280_convert_raw_temp(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
    ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t bmp280_convert_raw_pres(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
    var2 = var2 + (((int64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
    if (var1 == 0)
    {
    return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
    return (uint32_t)p;
}

// Load the trim data that is hardcoded and store it for later
uint8_t load_trim_registers(){
	uint8_t trimdata[26];
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Read(
        i2c_address, 
        BMP280+1, 
        0x88, 
        1, 
        trimdata, 
        26, 
        100
    );

    if(ret != HAL_OK){
        return 0;
    }

    dig_T1 = (trimdata[1]<<8) | trimdata[0];
	dig_T2 = (trimdata[3]<<8) | trimdata[2];
	dig_T3 = (trimdata[5]<<8) | trimdata[4];
	dig_P1 = (trimdata[7]<<8) | trimdata[5];
	dig_P2 = (trimdata[9]<<8) | trimdata[6];
	dig_P3 = (trimdata[11]<<8) | trimdata[10];
	dig_P4 = (trimdata[13]<<8) | trimdata[12];
	dig_P5 = (trimdata[15]<<8) | trimdata[14];
	dig_P6 = (trimdata[17]<<8) | trimdata[16];
	dig_P7 = (trimdata[19]<<8) | trimdata[18];
	dig_P8 = (trimdata[21]<<8) | trimdata[20];
	dig_P9 = (trimdata[23]<<8) | trimdata[22];

    return 1;
}

I2C_HandleTypeDef * i2c_address;

uint8_t init_bmp280(I2C_HandleTypeDef *i2c_address_temp){

    HAL_StatusTypeDef ret;

    i2c_address = i2c_address_temp;

    // Check that the i2c device is there 
    uint8_t check = 0;
    ret = HAL_I2C_Mem_Read(i2c_address, BMP280+1, 0xD0, 1, &check, 1, 100);

    if(ret != HAL_OK || check != 0x58){
        printf("BMP280 initialization failed\n");
        return 0;
    }

    // Try to reset it 
    uint8_t reset_device1[] = {0xE0, 0xB6};
    HAL_StatusTypeDef ret1 = HAL_I2C_Mem_Write(i2c_address, BMP280, reset_device1[0], 1, &reset_device1[1], 1, 100);

    uint8_t ctrl_meas_register = 0b00000000;
    ctrl_meas_register |= 0b00000011; // set it to normal mode
    ctrl_meas_register |= 0b00100000; // enable temp measurement oversampling x1
    ctrl_meas_register |= 0b00001100; // set pressure oversampling to standard resolution 18bit/0.66 PA x4
    HAL_StatusTypeDef ret2 =  HAL_I2C_Mem_Write(i2c_address, BMP280, 0xF4, 1, &ctrl_meas_register, 1, 100);
    
    uint8_t config_register = 0b00000000;
    config_register |= 0b00000000; // Set standby time to 0.5mx, disable spi on last bit
    config_register |= 0b00000100; // Set filter to coefficient 2
    HAL_StatusTypeDef ret3 = HAL_I2C_Mem_Write(i2c_address, BMP280, 0xF5, 1, &config_register, 1, 100);

    printf("BMP280 initialized\n");
    HAL_Delay(100);
    uint8_t load_trims = load_trim_registers();

    return ret1 == HAL_OK && ret2 == HAL_OK && ret3 == HAL_OK && load_trims == 1;
}

float bmp280_preassure_float(){
    uint8_t retrieved_data[] = {0,0,0};

    // Read 
    //  press_msb
    //  press_lsb
    //  press_xlsb
    HAL_I2C_Mem_Read(
        i2c_address, 
        BMP280+1, 
        0xF7, 
        1, 
        retrieved_data, 
        3, 
        100
    );

    // printf("PRESS %d %d %d \n", retrieved_data[0], retrieved_data[1], retrieved_data[2]);

    int32_t combined_pres = ((int32_t)retrieved_data[0] << 12) | ((int32_t)retrieved_data[1]) >> 4| ((int32_t) retrieved_data[1] >> 4);

    // printf("combined_pres: %ld\n", combined_pres);
    float pressure = ((float) bmp280_convert_raw_pres(combined_pres))/256.0;
    printf("pressure: %f\n", pressure/100);
    return pressure/100;
}

float bmp280_temperature_float(){
    uint8_t retrieved_data[] = {0,0,0};

    // Read 
    //  temp_msb
    //  temp_lsb
    //  temp_xlsb
    HAL_I2C_Mem_Read(
        i2c_address, 
        BMP280+1, 
        0xFA, 
        1, 
        retrieved_data, 
        3, 
        100
    );

    // printf("TEMP %d %d %d \n", retrieved_data[0], retrieved_data[1], retrieved_data[2]);
    
    int32_t combined_temp = ((int32_t)retrieved_data[0] << 12) | ((int32_t)retrieved_data[1]) >> 4| ((int32_t) retrieved_data[1] >> 4);

    // printf("combined_temp: %ld\n", combined_temp);
    float temperature = ((float) bmp280_convert_raw_temp(combined_temp))/100.0;
    printf("temperature: %f\n", temperature);
    return temperature;
}