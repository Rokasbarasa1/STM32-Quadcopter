#include "./bmp680.h"

#define BMP680_I2C_ID (0x76 << 1)

I2C_HandleTypeDef *i2c_handle;

uint8_t init_bmp680(I2C_HandleTypeDef *i2c_handle_temp){
    i2c_handle = i2c_handle_temp;

    printf("BMP680 initialized\n");
    return 1;
}