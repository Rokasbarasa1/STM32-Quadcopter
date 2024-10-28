#include "./dshot600.h"
#include <stdio.h>

// Inspired sources:
// A good guide on dshot600 protocol: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
// A good implementation of dshot600: https://github.com/gueei/DShot-Arduino
// A good way of implementing dshot600 on arduino: https://arduino.stackexchange.com/questions/43851/dshot-implementation-on-arduino-esc-protocol

uint16_t dshot600_motor_data_packet[DSHOT600_AMOUNT_OF_MOTORS];    // Data packet to be sent to motor
GPIO_TypeDef* dshot600_motor_port[DSHOT600_AMOUNT_OF_MOTORS];      // Port
uint16_t dshot600_motor_pin[DSHOT600_AMOUNT_OF_MOTORS];            // Pin
uint8_t dshot600_motor_list_size = 0;                              // How many motors are initialized in the array

// Initialize all the peripherals the motor needs to function
int8_t dshot600_add_motor(GPIO_TypeDef* motor_port, uint16_t motor_pin){
    if (dshot600_motor_list_size >= DSHOT600_AMOUNT_OF_MOTORS) {
        printf("dshot600 - Max number of motors reached\n");
        return -1;
    }

    // Save some data for the motor
    dshot600_motor_port[dshot600_motor_list_size] = motor_port;
    dshot600_motor_pin[dshot600_motor_list_size] = motor_pin;

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // This initializes the pin from scratch, no config for it is needed before hand
    if(motor_port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if(motor_port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if(motor_port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if(motor_port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();

    // Initialize the pin as output
    GPIO_InitStruct.Pin = motor_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(motor_port, &GPIO_InitStruct);
    // set the motor low as this is normal dshot600
    HAL_GPIO_WritePin(motor_port, motor_pin, 1);

    dshot600_motor_list_size++;
    return dshot600_motor_list_size-1;
}

// Computes the packet for sending it to motor. Can be used for commands also
void dshot600_set_throttle(uint16_t throttle, uint8_t motor_index_temp){
    // Reset the value
    dshot600_motor_data_packet[motor_index_temp] = 0;

    // Put the throttle info into the packet. Shift it for the telemetry bit
    dshot600_motor_data_packet[motor_index_temp] = throttle << 1;
    if(throttle < 48 && throttle > 0) dshot600_motor_data_packet[motor_index_temp] |= 1;
    
    // Calculate the 4 bit crc, then add it to the packet
    uint8_t crc = (dshot600_motor_data_packet[motor_index_temp] ^ (dshot600_motor_data_packet[motor_index_temp] >> 4) ^ (dshot600_motor_data_packet[motor_index_temp] >> 8)) & 0x0F;
    dshot600_motor_data_packet[motor_index_temp] = (dshot600_motor_data_packet[motor_index_temp] << 4) | crc;
}

// Old implementation that is just a tiny bit too fast or too slow compared to what the signal needs to be
// Keeping for performance efficient code without assembly in the future
// uint8_t dshot600_send_command(uint8_t motor_index){
//     CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable DWT
//     DWT->CYCCNT = 0;                                // Reset the cycle counter
//     DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

//     uint32_t start_cycles = DWT->CYCCNT;
//     for (uint8_t i = 0; i < 16; i++){
//         delay_until(start_cycles, total_cycles);  // Ensure the last bit finished exactly at 1.67 µs
//         start_cycles = DWT->CYCCNT;
//         dshot600_motor_port[motor_index]->BSRR = dshot600_motor_pin[motor_index]; // Set high
//         delay_until(start_cycles, middle_cycles);
//         dshot600_motor_port[motor_index]->BSRR = dshot600_motor_pin[motor_index] << (16 * !(0x01 & (dshot600_motor_data_packet[motor_index] >> (15 - i)))); // Take the first most significant bit and shift it to the lsb position
//         delay_until(start_cycles, middle_cycles + middle_cycles);
//         dshot600_motor_port[motor_index]->BSRR = dshot600_motor_pin[motor_index] << 16; // Set low
//     }
//     return 1;
// }

// Send throttle that was set to all motors
// Has to be called at 500Hz or more otherwise the motor wont do anything
void dshot600_send_all_motor_data(){
    for (uint8_t i = 0; i < dshot600_motor_list_size; i++) dshot600_send_command(i);
}

// Send data packet to the motor
void dshot600_send_command(uint8_t motor_index) {
    uint16_t packet = dshot600_motor_data_packet[motor_index];
    uint32_t pin_mask = dshot600_motor_pin[motor_index];
    GPIO_TypeDef* port = dshot600_motor_port[motor_index];
    uint32_t gpio_bsrr_addr = (uint32_t)&(port->BSRR); // Use the register of the port 
    
    printf("From dshot600 command\n");
    __asm__ volatile (
        // Setup registers
        "MOV r0, %[gpio_bsrr_addr]    \n" // r0 = gpio_bsrr_addr
        "MOV r1, %[pin_mask]          \n" // r1 = pin_mask
        "LSL r2, r1, #16              \n" // r2 = pin_mask << 16 (reset mask)
        "MOV r3, %[packet]            \n" // r3 = packet
        "MOVS r4, #16                 \n" // r4 = bit counter (16 bits)


        "1:                           \n" // Loop label
        "STR r1, [r0]                 \n" // Set pin high
        "TST r3, #(1 << 15)           \n" // Test MSB to if it is 1 or 0
        "BEQ 2f                       \n" // If zero, branch to 2



        // Bit 1 Handling
        // How many NOP for bit 1 while it is HIGH
        ".rept 110                     \n"
        "NOP                          \n"
        ".endr                        \n"
        "STR r2, [r0]                 \n" // Set pin low
        // How many NOP for bit 1 while it is LOW
        ".rept 36                     \n"
        "NOP                          \n"
        ".endr                        \n"
        "B 3f                         \n" // Branch to next_bit



        // Bit 0 Handling
        "2:                           \n"
        // How many NOP for bit 0 while it is HIGH
        ".rept 47       \n"
        "NOP                          \n"
        ".endr                        \n"
        "STR r2, [r0]                 \n" // Set pin low
        // How many NOP for bit 0 while it is LOW
        ".rept 100        \n"
        "NOP                          \n"
        ".endr                        \n"



        "3:                           \n" // next_bit label
        "LSL r3, r3, #1               \n" // Shift packet
        "SUBS r4, r4, #1              \n" // Decrement bit counter
        "BNE 1b                       \n" // Loop back if not zero
        :
        : [gpio_bsrr_addr] "r" (gpio_bsrr_addr),
          [pin_mask] "r" (pin_mask),
          [packet] "r" (packet)
        : "r0", "r1", "r2", "r3", "r4", "memory"
    );
}

// ----------------------------------------------------------------------------------------- Example use of driver
// uint8_t motor_BL;
// uint8_t motor_BR;
// uint8_t motor_FR;
// uint8_t motor_FL;

// // Optional 
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
//     if(htim->Instance == TIM3){
//         dshot600_send_all_motor_data();
//     }
// }

// int main(void){
//     motor_FL = dshot600_add_motor(GPIOA, GPIO_PIN_1);
//     motor_FR = dshot600_add_motor(GPIOB, GPIO_PIN_10);
//     motor_BL = dshot600_add_motor(GPIOA, GPIO_PIN_8);
//     motor_BR = dshot600_add_motor(GPIOA, GPIO_PIN_11);
    
//     while(1){
//         dshot600_set_throttle(100, motor_BL);
//         dshot600_set_throttle(100, motor_BR);
//         dshot600_set_throttle(100, motor_FR);
//         dshot600_set_throttle(100, motor_FL);

//         // Loop must run at least at 500Hz if calling without a timer
//         dshot600_send_all_motor_data();
//     }

//     return 0;
// }