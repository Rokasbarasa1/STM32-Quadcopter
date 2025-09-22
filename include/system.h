#pragma once
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern DMA_HandleTypeDef hdma_spi3_tx;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

int system_main(void);