#include "./bdshot600.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "../utils/math_constants.h"

// Inspired sources:
// A good guide on dshot600 protocol: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
// A good implementation of dshot600: https://github.com/gueei/DShot-Arduino
// A good way of implementing dshot600 on arduino: https://arduino.stackexchange.com/questions/43851/dshot-implementation-on-arduino-esc-protocol
// A not so good implementation of bdshot600 but still gives clues: https://github.com/bird-sanctuary/arduino-bi-directional-dshot
// A decent explanation of return signal on bdshot600: https://github.com/cinderblock/AVR/blob/master/AVR%2B%2B/BDShot.cpp

#define BDSHOT600_CAPTURE_COMPARE_CYCLES_SIZE 20                                                                               // How many rounds of capture to do for the response
uint16_t bdshot600_motor_data_packet[BDSHOT600_AMOUNT_OF_MOTORS];                                                              // Data packet to be sent to motor
GPIO_TypeDef* bdshot600_motor_ports[BDSHOT600_AMOUNT_OF_MOTORS];                                                               // Port
uint16_t bdshot600_motor_pins[BDSHOT600_AMOUNT_OF_MOTORS];                                                                     // Pin
TIM_HandleTypeDef bdshot600_timer[BDSHOT600_AMOUNT_OF_MOTORS];                                                                 // Timer struct
TIM_TypeDef* bdshot600_timer_id[BDSHOT600_AMOUNT_OF_MOTORS];                                                                   // TImer id (ex. TIM2)
TIM_IC_InitTypeDef bdshot600_timer_channel[BDSHOT600_AMOUNT_OF_MOTORS];                                                        // Timer channel struct
uint32_t bdshot600_timer_channel_id[BDSHOT600_AMOUNT_OF_MOTORS];                                                               // Timer channel id  (ex. TIM_CHANNEL_2)
uint32_t bdshot600_timer_channel_flag[BDSHOT600_AMOUNT_OF_MOTORS];                                                             // Timer channel edge detect flag. For optimizing
uint8_t bdshot600_motor_input_pin_alternate_mode[BDSHOT600_AMOUNT_OF_MOTORS];                                                  // Pin mode for timer functionality. For optimizing
volatile uint16_t bdshot600_motor_response_clock_cycles[BDSHOT600_AMOUNT_OF_MOTORS][BDSHOT600_CAPTURE_COMPARE_CYCLES_SIZE];    // Response from motor measured in clock cycles
uint16_t bdshot600_motor_period_us[BDSHOT600_AMOUNT_OF_MOTORS];                                                                // Motor period in us from response
float bdshot600_motor_frequency[BDSHOT600_AMOUNT_OF_MOTORS];                                                                   // Motor frequency from response
float bdshot600_motor_rpm[BDSHOT600_AMOUNT_OF_MOTORS];                                                                         // Motor rpm from response
uint8_t bdshot600_motor_list_size = 0;                                                                                         // How many motors are initialized in the array
volatile uint8_t bdshot600_busy_flag = 0;                                                                                      // To know when data is being written to the clock cycle array by the driver
volatile uint8_t bdshot600_conversion_finished = 0;                                                                            // Status flag to remember that data is stale

uint8_t bdshot600_number_of_rotor_pole_pairs = 0;
float bdshot600_number_of_rotor_pole_pairs_division_precalculated = 0;

// Reverse GCR mapping table
// The zero returns are not important
const uint8_t reverse_gcr_map[32] = {
    0, // 0
    0, // 1
    0, // 1
    0, // 3
    0, // 4
    0, // 5
    0, // 6
    0, // 7
    0, // 8
    0x9, // 9 -> 9
    0xA, // A -> A
    0xB, // B -> B
    0, // C
    0xD, // D -> D
    0xE, // E -> E
    0xF, // F -> F
    0, // 10
    0, // 11
    0x2, // 12 -> 2
    0x3, // 13 -> 3
    0, // 14
    0x5, // 15 -> 5
    0x6, // 16 -> 6
    0x7, // 17 -> 7
    0, // 18
    0x0, // 19 -> 0
    0x8, // 1A -> 8
    0x1, // 1B -> 1
    0, // 1C
    0x4, // 1D -> 4
    0xC, // 1E -> C
    0, // 1F
};

// Initialize all the peripherals the motor needs to function
int8_t bdshot600_add_motor(GPIO_TypeDef* motor_port, uint16_t motor_pin, TIM_TypeDef* timer_id, uint32_t timer_channel_id){
    if (bdshot600_motor_list_size >= BDSHOT600_AMOUNT_OF_MOTORS) {
        printf("bdshot600 - Max number of motors reached\n");
        return -1;
    }

    // Save some data for the motor
    bdshot600_motor_ports[bdshot600_motor_list_size] = motor_port;
    bdshot600_motor_pins[bdshot600_motor_list_size] = motor_pin;
    bdshot600_timer_id[bdshot600_motor_list_size] = timer_id;
    bdshot600_timer_channel_id[bdshot600_motor_list_size] = timer_channel_id;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_StatusTypeDef ret;

    // This initializes the pin from scratch, no config for it is needed before hand
    if(motor_port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if(motor_port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if(motor_port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if(motor_port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();

    // Initialize the clock for the timer and set the alternate pin mode 
    if(timer_id == TIM1){
        __HAL_RCC_TIM1_CLK_ENABLE();
        bdshot600_motor_input_pin_alternate_mode[bdshot600_motor_list_size] = GPIO_AF1_TIM1;
    }else if(timer_id == TIM2){
        __HAL_RCC_TIM2_CLK_ENABLE();
        bdshot600_motor_input_pin_alternate_mode[bdshot600_motor_list_size] = GPIO_AF1_TIM2;
    }else if(timer_id == TIM3){
        __HAL_RCC_TIM3_CLK_ENABLE();
        bdshot600_motor_input_pin_alternate_mode[bdshot600_motor_list_size] = GPIO_AF2_TIM3;
    }else if(timer_id == TIM4){
        __HAL_RCC_TIM4_CLK_ENABLE();
        bdshot600_motor_input_pin_alternate_mode[bdshot600_motor_list_size] = GPIO_AF2_TIM4;
    }else if(timer_id == TIM5){
        __HAL_RCC_TIM5_CLK_ENABLE();
        bdshot600_motor_input_pin_alternate_mode[bdshot600_motor_list_size] = GPIO_AF2_TIM5;
    }

    // Set timer channel flag to check
    if(timer_channel_id == TIM_CHANNEL_1){
        bdshot600_timer_channel_flag[bdshot600_motor_list_size] = TIM_FLAG_CC1;
    }else if(timer_channel_id == TIM_CHANNEL_2){
        bdshot600_timer_channel_flag[bdshot600_motor_list_size] = TIM_FLAG_CC2;
    }else if(timer_channel_id == TIM_CHANNEL_3){
        bdshot600_timer_channel_flag[bdshot600_motor_list_size] = TIM_FLAG_CC3;
    }else if(timer_channel_id == TIM_CHANNEL_4){
        bdshot600_timer_channel_flag[bdshot600_motor_list_size] = TIM_FLAG_CC4;
    }

    // Set interrupts
    // The interrupt timer triggering is not used but this is essential 
    // to get it to work even without interrupts.
    if(timer_id == TIM1){
        HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);  // Set the priority (can adjust values)
        HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);  // Enable the interrupt for TIM1
    }else if(timer_id == TIM2){
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);  // Set the priority (can adjust values)
        HAL_NVIC_EnableIRQ(TIM2_IRQn);  // Enable the interrupt for TIM1
    }else if(timer_id == TIM3){
        HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);  // Set the priority (can adjust values)
        HAL_NVIC_EnableIRQ(TIM3_IRQn);  // Enable the interrupt for TIM1
    }else if(timer_id == TIM4){
        HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);  // Set the priority (can adjust values)
        HAL_NVIC_EnableIRQ(TIM4_IRQn);  // Enable the interrupt for TIM1
    }else if(timer_id == TIM5){
        HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);  // Set the priority (can adjust values)
        HAL_NVIC_EnableIRQ(TIM5_IRQn);  // Enable the interrupt for TIM1
    }

    // Initialize the pin as output
    GPIO_InitStruct.Pin = motor_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(motor_port, &GPIO_InitStruct);
    // set the motor high as this is normal bdshot600. It is high by default
    HAL_GPIO_WritePin(motor_port, motor_pin, 1);

    //Configure the timer
    bdshot600_timer[bdshot600_motor_list_size].Instance = timer_id;
    bdshot600_timer[bdshot600_motor_list_size].Init.Prescaler = 0;  // To divide the clock frequency and get accurate timing
    bdshot600_timer[bdshot600_motor_list_size].Init.CounterMode = TIM_COUNTERMODE_UP;
    bdshot600_timer[bdshot600_motor_list_size].Init.Period = 5100-1;  // Maximum count to measure long pulses
    bdshot600_timer[bdshot600_motor_list_size].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    bdshot600_timer[bdshot600_motor_list_size].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if((ret = HAL_TIM_IC_Init(&bdshot600_timer[bdshot600_motor_list_size])) != HAL_OK){
        printf("bdshot600_add_motor - error, failed to initialize timer: %02x\n", ret);
        return -1;
    }

    bdshot600_timer_channel[bdshot600_motor_list_size].ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;  // Rising edge to start
    bdshot600_timer_channel[bdshot600_motor_list_size].ICSelection = TIM_ICSELECTION_DIRECTTI;
    bdshot600_timer_channel[bdshot600_motor_list_size].ICPrescaler = TIM_ICPSC_DIV1;
    bdshot600_timer_channel[bdshot600_motor_list_size].ICFilter = 0;
    if((ret = HAL_TIM_IC_ConfigChannel(&bdshot600_timer[bdshot600_motor_list_size], &bdshot600_timer_channel[bdshot600_motor_list_size], timer_channel_id)) != HAL_OK){
        printf("bdshot600_add_motor - error, failed to initialize timer channel: %02x\n", ret);
        return -1;
    }


    bdshot600_motor_list_size++;
    return bdshot600_motor_list_size-1;
}

uint8_t bdshot600_set_rotor_poles_amount(uint8_t rotor_poles){
    // check if number is odd and if it is quit
    if(rotor_poles % 2 == 1){
        printf("bdshot600_set_rotor_poles_amount - rotor poles amount is odd, it should be even");
        return 0;
    }
    bdshot600_number_of_rotor_pole_pairs = rotor_poles / 2; // Pairs so we half them

    bdshot600_number_of_rotor_pole_pairs_division_precalculated = 1.0f / bdshot600_number_of_rotor_pole_pairs;

    return 1;
}

// Computes the packet for sending it to motor. Can be used for commands also
void bdshot600_set_throttle(uint16_t throttle, uint8_t motor_index_temp){
    // Reset the value
    bdshot600_motor_data_packet[motor_index_temp] = 0;

    // Put the throttle info into the packet. Shift it for the telemetry bit
    bdshot600_motor_data_packet[motor_index_temp] = throttle << 1; 
    if(throttle < 48 && throttle > 0) bdshot600_motor_data_packet[motor_index_temp] |= 1;

    // Calculate the 4 bit crc and then invert it, then add it to the packet
    uint8_t crc = (~(bdshot600_motor_data_packet[motor_index_temp] ^ (bdshot600_motor_data_packet[motor_index_temp] >> 4) ^ (bdshot600_motor_data_packet[motor_index_temp] >> 8))) & 0x0F;
    bdshot600_motor_data_packet[motor_index_temp] = (bdshot600_motor_data_packet[motor_index_temp] << 4) | crc;
}

// Send throttle that was set to all motors
// Has to be called at 500Hz or more otherwise the motor wont do anything
void bdshot600_send_all_motor_data(){
    // Set as busy so reading from the data arrays does not happen by the main loop
    bdshot600_busy_flag = 1;
    bdshot600_conversion_finished = 0;
    for (uint8_t i = 0; i < bdshot600_motor_list_size; i++) bdshot600_send_command(i);
    bdshot600_busy_flag = 0;
}

// Check if the driver is busy sending and getting data from motors
uint8_t bdshot600_get_busy(){
    return bdshot600_busy_flag;
}

// Send data packet and wait for response from the motor
void bdshot600_send_command(uint8_t motor_index){
    // Declare all the variables before starting time sensitive stuff
    uint16_t packet = bdshot600_motor_data_packet[motor_index];
    uint32_t pin_mask = bdshot600_motor_pins[motor_index];
    GPIO_TypeDef* port = bdshot600_motor_ports[motor_index];
    uint32_t gpio_bsrr_addr = (uint32_t)&(port->BSRR); // Use the register of the port 
    uint16_t value = 0;
    uint16_t previous_value = 0;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Set all array indexes to 0
    memset(
        bdshot600_motor_response_clock_cycles[motor_index], 
        0, 
        BDSHOT600_CAPTURE_COMPARE_CYCLES_SIZE * sizeof(uint16_t)
    );

    // This is just putting out the throttle data
    // Inspired by: https://github.com/gueei/DShot-Arduino
    __asm__ volatile (
        // Setup registers
        "MOV r0, %[gpio_bsrr_addr]    \n" // r0 = gpio_bsrr_addr
        "MOV r1, %[pin_mask]          \n" // r1 = pin_mask
        "LSL r2, r1, #16              \n" // r2 = pin_mask << 16 (reset mask)
        "MOV r3, %[packet]            \n" // r3 = packet
        "MOVS r4, #16                 \n" // r4 = bit counter (16 bits)


        "1:                           \n" // Loop
        "STR r2, [r0]                 \n" // Set pin low
        "TST r3, #(1 << 15)           \n" // Test MSB to if it is 1 or 0
        "BEQ 2f                       \n" // If 0, go to branch 2



        // Bit 1 Handling
        // How many NOP for bit 1 while it is LOW
        ".rept 110                     \n"
        "NOP                          \n"
        ".endr                        \n"
        "STR r1, [r0]                 \n" // Set pin high
        // How many NOP for bit 1 while it is HIGH
        ".rept 36        \n"
        "NOP                          \n"
        ".endr                        \n"
        "B 3f                         \n" // Branch to next_bit



        // Bit 0 Handling
        "2:                           \n"
        // How many NOP for bit 0 while it is LOW 
        ".rept 47                     \n"
        "NOP                          \n"
        ".endr                        \n"
        "STR r1, [r0]                 \n" // Set pin high
        // How many NOP for bit 0 while it is HIGH
        ".rept 100                     \n"
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

    // Make the pin a input capture compare pin
    GPIO_InitStruct.Pin = bdshot600_motor_pins[motor_index];
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = bdshot600_motor_input_pin_alternate_mode[motor_index];
    HAL_GPIO_Init(bdshot600_motor_ports[motor_index], &GPIO_InitStruct);

    // Wait just enough to capture first edge
    // Start capturing on the input channel. It does not need to be interrupt one
    HAL_TIM_IC_Start(&bdshot600_timer[motor_index], bdshot600_timer_channel_id[motor_index]);
    // Clear capture flag (CC1)
    __HAL_TIM_CLEAR_FLAG(&bdshot600_timer[motor_index], bdshot600_timer_channel_flag[motor_index]); // Clear edge capture event flag
    __HAL_TIM_CLEAR_FLAG(&bdshot600_timer[motor_index], TIM_FLAG_UPDATE);  // Clear update flag (overflow)
    __HAL_TIM_CLEAR_FLAG(&bdshot600_timer[motor_index], TIM_FLAG_CC1OF);  // Clear overcapture flag, even if not used
    
    for( uint8_t i = 0; i < BDSHOT600_CAPTURE_COMPARE_CYCLES_SIZE; i++) {
        // Wait for the counter overflow or the capture flag to be set.
        while (
            !(bdshot600_timer[motor_index].Instance->SR & bdshot600_timer_channel_flag[motor_index]) && 
            !(bdshot600_timer[motor_index].Instance->SR & TIM_FLAG_UPDATE)
        );

        // Check if it overflew. If yes break. Overflow means the timer ran out of time
        if (bdshot600_timer[motor_index].Instance->SR & TIM_FLAG_UPDATE) break;

        // Get the value
        value = HAL_TIM_ReadCapturedValue(&bdshot600_timer[motor_index], bdshot600_timer_channel_id[motor_index]);
        
        // Clear the input capture flag
        __HAL_TIM_CLEAR_FLAG(&bdshot600_timer[motor_index], bdshot600_timer_channel_flag[motor_index]);

        // Do not calculate the cycles if there is no previous value
        if(previous_value != 0) bdshot600_motor_response_clock_cycles[motor_index][i-1] = value - previous_value;
        previous_value = value;

        // If you want to see what edges are captured use this and hook up two probes
        // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
        // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
    }

    // Stop the timer as it is not needed anymore
    HAL_TIM_IC_Stop(&bdshot600_timer[motor_index], bdshot600_timer_channel_id[motor_index]);

    // Set pin as output again
    GPIO_InitStruct.Pin = bdshot600_motor_pins[motor_index];
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(bdshot600_motor_ports[motor_index], &GPIO_InitStruct);
    HAL_GPIO_WritePin(bdshot600_motor_ports[motor_index], bdshot600_motor_pins[motor_index], 1);

    /*
        0 (48) Throttle signal looks like this:
        
        -----__-_-__-_-__-_-_-___-----
        
        Parsed:
        001010010100101010001

        This is because what is valuable in that signal is:
        __-_-__-_-__-_-_-___-


        The signal is 750kbit/s so each bit after first falling edge is (600/750 * 1.67=) 1.336us in length. 
        On my hardware that is ~90 clock cycles

        My method does not capture all the bits, but it captures all the edge changes.
        So i can assume that if there are not 21 bits captured the remaining bits will be 
        whatever the bit value * (21-actual amount of bits captured). Works every time.
        
        First falling edge is always 0 and is always the most significant bit. What came before the first 
        edge is useless, if you think about it.
    */
}

// Convert all responses from motors to period, frequency and rpm
uint8_t bdshot600_convert_all_responses(){
    if(!bdshot600_conversion_finished){
        for (uint8_t i = 0; i < bdshot600_motor_list_size; i++){ 
            if (!bdshot600_convert_response_to_data(i)){
                // printf("Failed to convert\n");
                return 0;
            }
        }
    }

    bdshot600_conversion_finished = 1;
    return 1;
}

// Convert the response from the motor to real data of period, frequency and rpm
// This is a separate function form the interrupt to keep the interrupt as short as possible
uint8_t bdshot600_convert_response_to_data(uint8_t motor_index){

    // Copy the bits to a new array
    uint16_t response_clock_cycles_copy[20];
    // Wait for the the interrupt to stop writing data just in case. To not read broken stuff.
    while(bdshot600_get_busy()); 
    memcpy(
        response_clock_cycles_copy, 
        bdshot600_motor_response_clock_cycles[motor_index], 
        20 * sizeof(uint16_t)
    );

    // Check if any clock cycles captured
    if(response_clock_cycles_copy[0] == 0){
        // printf("First index is 0\n");
        return 0;
    }

    // Convert to bits
    uint8_t bit_amount = 0;
    // The first bit will always be a zero and that one is not even shown so first bit is actually 1
    uint8_t bit_value = 0; // Keep track of what bit value we are counting right now
    volatile uint32_t response_in_bits = 0;

    // Loop over the clock cycles amounts and see how many bits in length each block of these cycles is
    for (uint8_t i = 0; i < BDSHOT600_CAPTURE_COMPARE_CYCLES_SIZE; i++){
        // If the clock cycle value is 0 at any point, break the loop
        // if(counter_compare_capture_cycles[i] == 0) break;
        if(response_clock_cycles_copy[i] == 0) break;


        // Rounding to nearest integer is important here. Default integer rounding down is not good.
        uint8_t number_of_bits_in_this_clock_cycle_capture = (uint8_t)round((float)response_clock_cycles_copy[i]/125.0); // 125 is a good number between 117 and 134

        // Place the number of bits in the response
        for (size_t j = 0; j < number_of_bits_in_this_clock_cycle_capture; j++){
            // Shift it to the left by one bit
            response_in_bits <<= 1;
            response_in_bits |= bit_value;
            bit_amount++;
        }

        // If this one was 0 then next one will be 1
        bit_value ^= 1; // Flip the LSB from 0 to 1 and vice versa
    }

    // The edge detection using input capture compare does not capture the very last bits
    // Make a good assumption on them instead
    // Check how many bits are missing from 21 bit total and add the difference using the current bit value
    if(bit_amount < 21){
        for (uint8_t i = 0; i < 21-bit_amount; i++){
            response_in_bits <<= 1;
            response_in_bits |= bit_value;
        }
    }

    // -------------------------------------------------------------------Result of getting the bits from the signal
    // 235 133 126 118 239 126 117 122 231 125 118 122 121 126 353 for 100Mhz
    // 179 97 92 91 179 91 92 100 170 91 100 94 89 91 265 0 0 0 0 0  for 75Mhz
    // becomes
    // 00101 00101001 01010001
    // data2 = response_in_bits;


    response_in_bits = (response_in_bits ^ (response_in_bits >> 1));

    // -------------------------------------------------------------------Result of mapping the value from 21bits to 20bits
    // 00101 00101001 01010001
    // becomes
    //  0111 10111101 11111001
    // data2 = response_in_bits;
    // Not using MSb 21 from now


    // Get the 5 bit nibbles of the GCR
    uint8_t gcr_nibble_1 = (response_in_bits >> 15) & 0b11111; // 01111 -> 0xF
    uint8_t gcr_nibble_2 = (response_in_bits >> 10) & 0b11111; // 01111 -> 0xF
    uint8_t gcr_nibble_3 = (response_in_bits >> 5) & 0b11111;  // 01111 -> 0xF
    uint8_t gcr_nibble_4 = response_in_bits & 0b11111;         // 11001 -> 0x19

    // Turn the GCR nibbles into 4 bit nibbles using the converter array
    uint8_t a_16bit_nibble_1 = reverse_gcr_map[gcr_nibble_1];  // Converted: 0xF -> 1111
    uint8_t a_16bit_nibble_2 = reverse_gcr_map[gcr_nibble_2];  // Converted: 0xF -> 1111
    uint8_t a_16bit_nibble_3 = reverse_gcr_map[gcr_nibble_3];  // Converted: 0xF -> 1111
    uint8_t a_16bit_nibble_4 = reverse_gcr_map[gcr_nibble_4];  // Converted: 0x0 -> 0000


    // Combine the bits to one 16 bit value
    uint16_t payload_16bit = 
        (a_16bit_nibble_1 << 12) | 
        (a_16bit_nibble_2 << 8) | 
        (a_16bit_nibble_3 << 4) | 
        a_16bit_nibble_4;
    
    // -------------------------------------------------------------------Result of converting the GCR to 16 bit data
    // 0111 10111101 11111001
    // becomes
    // 11111111 11110000
    // eeeppppp ppppcccc // e - exponent p - payload c - crc
    // data2 = payload_16bit;


    // Leave only the 12 bits of the payload and exponent
    uint16_t payload_16bit_only_exponent_and_payload = (payload_16bit >> 4) & 0b111111111111;
    // Leave only the 9 bits of the payload and exponent
    uint16_t payload_16bit_only_payload = (payload_16bit >> 4) & 0b111111111;
    // Leave only the 3 bits of the exponent
    uint16_t payload_16bit_only_exponent = (payload_16bit >> 13) & 0b111;
    // Leave only the 4 bits of the crc
    uint16_t payload_16bit_only_crc = payload_16bit & 0b1111;

    // Calculate the new crc
    // The calculated crc is inverted. All the guides said not, but I found that it is indeed inverted.
    uint8_t calculated_crc = (~(payload_16bit_only_exponent_and_payload ^ (payload_16bit_only_exponent_and_payload >> 4) ^ (payload_16bit_only_exponent_and_payload >> 8))) & 0x0F;

    // Compare if the crc's are equal
    // If they are not equal, do not continue
    if(calculated_crc != payload_16bit_only_crc){

        // For debugging 
        // printf("Failed CRC \n");
        // for (int i = 3; i >= 0; i--) printf("%u", (calculated_crc >> i) & 1); 
        // printf(" != ");
        // for (int i = 3; i >= 0; i--) printf("%u", (payload_16bit_only_crc >> i) & 1); 
        // printf("\n");

        // Return 0 to indicate that something wrong happened durring conversion
        return 0;
    }

    // Get the period in us, frequency and rpm
    bdshot600_motor_period_us[motor_index] = payload_16bit_only_payload << payload_16bit_only_exponent; // payload is 9 bits and the exponent can move it 7 bits to the left making it a 16 bit value at max
    // 65408 means the motor is not spinning. Need to set everything to zero when this happens
    if(bdshot600_motor_period_us[motor_index] == 65408){
        bdshot600_motor_frequency[motor_index] = 0;
    }else{
        // frequency has to be divided by motor pole pairs to get the actual mechanical frequency
        bdshot600_motor_frequency[motor_index] = (1.0/((float)bdshot600_motor_period_us[motor_index] * MICROSECONDS_TO_SECONDS) * bdshot600_number_of_rotor_pole_pairs_division_precalculated);
    }
    bdshot600_motor_rpm[motor_index] = 60 * bdshot600_motor_frequency[motor_index];

    return 1;
}

// Motor period
uint16_t bdshot600_get_period_us(uint8_t motor_index){
    return bdshot600_motor_period_us[motor_index];
}

// Motor frequency
float bdshot600_get_frequency(uint8_t motor_index){
    return bdshot600_motor_frequency[motor_index];
}

// Motor rpm
float bdshot600_get_rpm(uint8_t motor_index){
    return bdshot600_motor_rpm[motor_index];
}

// ----------------------------------------------------------------------------------------- Example use of driver
// float motor_frequency[] = {0.0, 0.0, 0.0, 0.0};
// float motor_rpm[] = {0.0, 0.0, 0.0, 0.0};

// uint8_t motor_BL;
// uint8_t motor_BR;
// uint8_t motor_FR;
// uint8_t motor_FL;

// // Optional 
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
//     if(htim->Instance == TIM3){
//         bdshot600_send_all_motor_data();
//     }
// }

// int main(void){
//     motor_FL = bdshot600_add_motor(GPIOA, GPIO_PIN_1,  TIM2, TIM_CHANNEL_2);
//     motor_FR = bdshot600_add_motor(GPIOB, GPIO_PIN_10, TIM2, TIM_CHANNEL_3);
//     motor_BL = bdshot600_add_motor(GPIOA, GPIO_PIN_8,  TIM1, TIM_CHANNEL_1);
//     motor_BR = bdshot600_add_motor(GPIOA, GPIO_PIN_11, TIM1, TIM_CHANNEL_4);
    
//     while(1){
//         if(bdshot600_convert_all_responses(1)){

//             // Motor 0 - BL, 1 - BR, 2 - FR, 3 - FL
//             motor_frequency[0] = bdshot600_get_frequency(motor_BL);
//             motor_frequency[1] = bdshot600_get_frequency(motor_BR);
//             motor_frequency[2] = bdshot600_get_frequency(motor_FR);
//             motor_frequency[3] = bdshot600_get_frequency(motor_FL);

//             motor_rpm[0] = bdshot600_get_rpm(motor_BL); 
//             motor_rpm[1] = bdshot600_get_rpm(motor_BR);
//             motor_rpm[2] = bdshot600_get_rpm(motor_FR);
//             motor_rpm[3] = bdshot600_get_rpm(motor_FL);
//         }

//         bdshot600_set_throttle(100, motor_BL);
//         bdshot600_set_throttle(100, motor_BR);
//         bdshot600_set_throttle(100, motor_FR);
//         bdshot600_set_throttle(100, motor_FL);

//         // Loop must run at least at 500Hz if calling without a timer
//         bdshot600_send_all_motor_data();
//     }

//     return 0;
// }