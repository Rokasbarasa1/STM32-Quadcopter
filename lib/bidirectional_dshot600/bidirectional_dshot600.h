#pragma once
#include <stdio.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

uint8_t dshot600_init(uint32_t clock_frequency);
uint8_t dshot600_add_motor(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint8_t dshot600_start_timer(TIM_HandleTypeDef *timer);
uint8_t dshot600_send_all_motor_data();
uint8_t dshot600_set_throttle(uint16_t throttle, uint8_t motor_index_temp);
uint8_t dshot600_send_command(uint8_t motor_index);