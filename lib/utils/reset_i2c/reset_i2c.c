#include "./reset_i2c.h"

void I2C_Bus_Reset(GPIO_TypeDef *GPIO_SCL, uint16_t SCL_Pin, GPIO_TypeDef *GPIO_SDA, uint16_t SDA_Pin) {
    // Configure SCL and SDA pins as GPIO outputs
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure SCL pin as output
    GPIO_InitStruct.Pin = SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIO_SCL, &GPIO_InitStruct);

    // Configure SDA pin as input to read its state
    GPIO_InitStruct.Pin = SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIO_SDA, &GPIO_InitStruct);

    // Toggle SCL until SDA is released
    for (int i = 0; i < 10; i++) { // Maximum attempts to recover
        if (HAL_GPIO_ReadPin(GPIO_SDA, SDA_Pin) == GPIO_PIN_SET) {
            break; // SDA line is released
        }

        // Toggle SCL
        HAL_GPIO_WritePin(GPIO_SCL, SCL_Pin, GPIO_PIN_SET); // SCL High
        HAL_Delay(1); // Small delay
        HAL_GPIO_WritePin(GPIO_SCL, SCL_Pin, GPIO_PIN_RESET); // SCL Low
        HAL_Delay(1); // Small delay
    }

    // Send a STOP condition manually
    GPIO_InitStruct.Pin = SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIO_SDA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIO_SDA, SDA_Pin, GPIO_PIN_SET); // SDA High
    HAL_GPIO_WritePin(GPIO_SCL, SCL_Pin, GPIO_PIN_SET); // SCL High
    HAL_Delay(1); // Small delay

    // Restore SCL and SDA pins to alternate function mode for I2C
    GPIO_InitStruct.Pin = SCL_Pin | SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIO_SCL, &GPIO_InitStruct);
}