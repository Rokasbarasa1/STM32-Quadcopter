#pragma once
#include "stm32f4xx_hal.h"

// This driver assumes that you have a 100Mhz HCLK clock
// If you dont you need to flash an arduino uno or something and comparing output of this driver to that uno align the NOP amounts in assembly code
// If running at 75MHz look for old versions of this driver in git

// How many motors can this library handle. Can be any number really, but it cost memory to allocate all those arrays#endif
// Default is 4
#ifndef BDSHOT600_AMOUNT_OF_MOTORS
    #define BDSHOT600_AMOUNT_OF_MOTORS 4
#endif

int8_t bdshot600_add_motor(GPIO_TypeDef* motor_port, uint16_t motor_pin, TIM_TypeDef* timer_id, uint32_t timer_channel_id);
void bdshot600_set_throttle(uint16_t throttle, uint8_t motor_index_temp);
void bdshot600_send_all_motor_data();
uint8_t bdshot600_get_busy();
void bdshot600_send_command(uint8_t motor_index);
uint8_t bdshot600_convert_all_responses();
uint8_t bdshot600_convert_response_to_data(uint8_t motor_index);
uint16_t bdshot600_get_period_us(uint8_t motor_index);
float bdshot600_get_frequency(uint8_t motor_index);
float bdshot600_get_rpm(uint8_t motor_index);