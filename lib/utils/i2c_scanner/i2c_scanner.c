#include "i2c_scanner.h"

void i2c_scan_bus(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef result;
    uint8_t i;
    printf("Scanning I2C bus...\n");

    for (i = 1; i < 127; i++) {  // 7-bit addresses range from 0x01 to 0x7E
        /*
         * The HAL expects the 8-bit address here, so shift left by 1
         * Then use HAL_I2C_IsDeviceReady() to check for an ACK
         */
        result = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(i << 1), 2, 5);
        if (result == HAL_OK) {
            printf("I2C device found at 0x%02X\r\n", i);
        }
    }

    printf("I2C scan done.\n");
}
