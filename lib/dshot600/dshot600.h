#pragma once
#include "stm32f4xx_hal.h"


// This driver assumes that you have a 75Mhz clock
int8_t dshot600_add_motor(GPIO_TypeDef* motor_port, uint16_t motor_pin);
void dshot600_set_throttle(uint16_t throttle, uint8_t motor_index_temp);
void dshot600_send_all_motor_data();
void dshot600_send_command(uint8_t motor_index);
