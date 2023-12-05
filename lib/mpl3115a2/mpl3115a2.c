#include "./mpl3115a2.h"

#define MPL3115A2_I2C_ID (0x76 << 1)

I2C_HandleTypeDef *i2c_handle;

uint8_t init_mpl3115a2(I2C_HandleTypeDef *i2c_handle_temp){
    i2c_handle = i2c_handle_temp;

    printf("MPL3115A2 initialized\n");
    return 1;
}