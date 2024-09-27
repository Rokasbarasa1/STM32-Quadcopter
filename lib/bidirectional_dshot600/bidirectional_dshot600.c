#include "./bidirectional_dshot600.h"

uint8_t calculate_dshot_crc(uint16_t packet) {
    uint8_t crc = 0;
    uint8_t crc_mask[] = {0x1, 0x3, 0x7, 0xF};  // Bitwise XOR masks for CRC calculation
    
    for (int i = 0; i < 12; i++) {
        if (packet & (1 << i)) {  // If the ith bit of the packet is 1
            crc ^= crc_mask[i % 4];  // XOR with corresponding crc_mask
        }
    }
    return crc;
}

// Function to delay in microseconds
void delay_cycles(uint32_t cycles) {
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

void delay_until(uint32_t start_cycles, uint32_t target_cycles) {
    while ((DWT->CYCCNT - start_cycles) < target_cycles);
}

void print_binary(uint16_t num) {
    for (int i = 15; i >= 0; i--) {  // Iterate from the most significant bit to the least significant bit
        printf("%d", (num >> i) & 1);  // Print the ith bit (shift and mask to get the bit)
    }
    printf("\n");  // New line after printing all bits
}

uint16_t motor_data_packet[20];
GPIO_TypeDef* motor_port[20];
uint16_t motor_pin[20];
uint8_t motor_list_size = 0;
uint8_t motor_settings_index;

uint32_t clock_frequency;
uint32_t total_cycles;
uint32_t middle_cycles;

uint8_t dshot600_init(uint32_t clock_frequency){
    total_cycles = (1722 * (clock_frequency/1000)) / 1000000; // 1670
    middle_cycles = (360 * (clock_frequency/1000)) / 1000000;  // 625 // 360 or 23 cycles

    total_cycles -= 1;
    printf("Total: %lu, Middle: %lu\n", total_cycles, middle_cycles);
    return 1;
}

uint8_t dshot600_add_motor(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
    motor_port[motor_list_size] = GPIOx;
    motor_pin[motor_list_size] = GPIO_Pin;

    motor_list_size++;
    
    return motor_list_size-1;
}

uint8_t dshot600_start_timer(TIM_HandleTypeDef *timer){
    HAL_TIM_Base_Start_IT(timer); /// Not sure why i have this function as it just calls the timer interrupt, but it looks nicer
    return 1;
}

uint8_t dshot600_send_all_motor_data(){
    for (uint8_t i = 0; i < motor_list_size; i++){
        dshot600_send_command(i);
        // printf("%d dshot600_send_all_motor_data\n", i);
    }
    return 1;
}

// Total signal length 26.7 / 2 = 13.35
uint8_t dshot600_set_throttle(uint16_t throttle, uint8_t motor_index_temp){
    // printf("Throttle: %d\n", throttle);
    motor_data_packet[motor_index_temp] = 0;

    motor_data_packet[motor_index_temp] = throttle << 1; // Put the throttle into the data. Put the throttle info into the packet
    if(throttle < 48 && throttle > 0) motor_data_packet[motor_index_temp] |= 1;
    
    uint8_t csum = 0;
    uint16_t csum_data = motor_data_packet[motor_index_temp];
    for (uint8_t i = 0; i < 3; i++){
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0xf;
    motor_data_packet[motor_index_temp] = (motor_data_packet[motor_index_temp] << 4) | csum;
    return 1;
}

// uint8_t dshot600_send_command(uint8_t motor_index){
//     CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable DWT
//     DWT->CYCCNT = 0;                                // Reset the cycle counter
//     DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

//     uint32_t start_cycles = DWT->CYCCNT;
//     for (uint8_t i = 0; i < 16; i++){
//         delay_until(start_cycles, total_cycles);  // Ensure the last bit finished exactly at 1.67 µs
//         start_cycles = DWT->CYCCNT;
//         motor_port[motor_index]->BSRR = motor_pin[motor_index]; // Set high
//         delay_until(start_cycles, middle_cycles);
//         motor_port[motor_index]->BSRR = motor_pin[motor_index] << (16 * !(0x01 & (motor_data_packet[motor_index] >> (15 - i)))); // Take the first most significant bit and shift it to the lsb position
//         delay_until(start_cycles, middle_cycles + middle_cycles);
//         motor_port[motor_index]->BSRR = motor_pin[motor_index] << 16; // Set low
//     }
//     return 1;
// }


__attribute__((section(".ramfunc"))) uint8_t dshot600_send_command(uint8_t motor_index) {
    uint16_t packet = motor_data_packet[motor_index];
    uint32_t pin_mask = motor_pin[motor_index];
    GPIO_TypeDef* GPIOx = motor_port[motor_index];
    uint32_t gpio_bsrr_addr = (uint32_t)&(GPIOx->BSRR);
    
    // Inspired by: https://github.com/gueei/DShot-Arduino
    __asm__ volatile (
        // Setup registers
        "MOV r0, %[gpio_bsrr_addr]    \n" // r0 = gpio_bsrr_addr
        "MOV r1, %[pin_mask]          \n" // r1 = pin_mask
        "LSL r2, r1, #16              \n" // r2 = pin_mask << 16 (reset mask)
        "MOV r3, %[packet]            \n" // r3 = packet
        "MOVS r4, #16                 \n" // r4 = bit counter (16 bits)

        "1:                           \n" // Loop label
        // Set pin high
        "STR r1, [r0]                 \n" // GPIOx->BSRR = pin_mask (set pin high)

        // Test the most significant bit (MSB)
        "TST r3, #(1 << 15)           \n" // Test bit 15
        "BEQ 2f                       \n" // If zero, branch to bit_is_zero

        // **'1' Bit Handling**
        // **High Time NOPs** (Adjust NOP count here)
        ".rept 85       \n"
        "NOP                          \n"
        ".endr                        \n"

        // Set pin low
        "STR r2, [r0]                 \n"

        // **Low Time NOPs** (Adjust NOP count here)
        ".rept 25        \n"
        "NOP                          \n"
        ".endr                        \n"

        "B 3f                         \n" // Branch to next_bit

        "2:                           \n" // bit_is_zero label
        // **'0' Bit Handling**
        // **High Time NOPs** (Adjust NOP count here)
        ".rept 36       \n"
        "NOP                          \n"
        ".endr                        \n"

        // Set pin low
        "STR r2, [r0]                 \n"

        // **Low Time NOPs** (Adjust NOP count here)
        ".rept 77        \n"
        "NOP                          \n"
        ".endr                        \n"

        "3:                           \n" // next_bit label
        // Shift packet
        "LSL r3, r3, #1               \n"
        // Decrement bit counter
        "SUBS r4, r4, #1              \n"
        // Loop back if not zero
        "BNE 1b                       \n"
        :
        : [gpio_bsrr_addr] "r" (gpio_bsrr_addr),
          [pin_mask] "r" (pin_mask),
          [packet] "r" (packet)
        : "r0", "r1", "r2", "r3", "r4", "memory"
    );

    return 1;
}