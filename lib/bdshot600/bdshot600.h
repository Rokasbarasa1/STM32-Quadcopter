#pragma once
#include "stm32f4xx_hal.h"

int8_t bdshot600_add_motor(GPIO_TypeDef* motor_port, uint16_t motor_pin, TIM_TypeDef* timer_id, uint32_t timer_channel_id);
void bdshot600_send_all_motor_data();
void bdshot600_set_throttle(uint16_t throttle, uint8_t motor_index_temp);
void bdshot600_send_command(uint8_t motor_index);
TIM_HandleTypeDef* bdshot600_get_motor_timer_pointer(uint8_t motor_index);

uint8_t bdshot600_get_busy();
uint32_t bdshot600_get_data2();


uint16_t bdshot600_get_period_us(uint8_t motor_index);
float bdshot600_get_frequency(uint8_t motor_index);
float bdshot600_get_rpm(uint8_t motor_index);
uint8_t bdshot600_convert_response_to_data(uint8_t motor_index);
uint8_t bdshot600_convert_all_responses();