#pragma once
#include "stm32f4xx_hal.h"


// This driver assumes that you have a 75Mhz clock
uint8_t dshot600_add_motor(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void dshot600_set_throttle(uint16_t throttle, uint8_t motor_index_temp);
void dshot600_send_all_motor_data();
void dshot600_send_command(uint8_t motor_index);
