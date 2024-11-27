#include "./bdshot600_dma.h"
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

#define one_bit_cycles 125 // 125 is a good number between 117 and 134
#define one_bit_cycles_half 63 // for rounding
#define BDSHOT600_DMA_AMOUNT_OF_MOTORS 4                                                                                                        // How many motors can this library handle. Can be any number really, but it cost memory to allocate all those arrays
#define BDSHOT600_DMA_CAPTURE_COMPARE_CYCLES_SIZE 20                                                                                            // How many rounds of capture to do for the response
#define BDSHOT600_DMA_CAPTURE_COMPARE_CYCLES_SIZE_DMA 20                                                                                        // It is good to have a second buffer to double buffer the data 
volatile uint16_t bdshot600_dma_motor_data_packet[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                              // Data to be sent to motor
volatile GPIO_TypeDef* bdshot600_dma_motor_ports[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                               // Port
volatile uint16_t bdshot600_dma_motor_pins[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                                     // Pin
volatile TIM_HandleTypeDef bdshot600_dma_timer[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                                 // Timer struct
volatile TIM_TypeDef* bdshot600_dma_timer_id[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                                   // TImer id
volatile TIM_IC_InitTypeDef bdshot600_dma_timer_channel[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                        // Timer channel struct
volatile uint32_t bdshot600_dma_timer_channel_id[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                               // Timer channel id
volatile uint32_t bdshot600_dma_timer_channel_flag[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                             // Timer channel edge detect flag
volatile uint8_t bdshot600_dma_motor_input_pin_alternate_mode[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                  // Pin mode for timer functionality
volatile DMA_HandleTypeDef bdshot600_dma_handle[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                                // DMA instance
volatile uint8_t bdshot600_dma_timer_reset[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                                     // Does this motor reset the timer counter before it starts again?
volatile uint8_t bdshot600_dma_motor_dma_status[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                                // Is the dma active or not? Set by manual check
volatile uint16_t bdshot600_dma_motor_response_clock_cycles[BDSHOT600_DMA_AMOUNT_OF_MOTORS][BDSHOT600_DMA_CAPTURE_COMPARE_CYCLES_SIZE];         // Response from motor measured in clock cycles
volatile uint16_t bdshot600_dma_motor_response_clock_cycles_dma[BDSHOT600_DMA_AMOUNT_OF_MOTORS][BDSHOT600_DMA_CAPTURE_COMPARE_CYCLES_SIZE_DMA]; // Buffer that the dma writes the raw clock cycle counts to
volatile uint16_t bdshot600_dma_motor_period_us[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                                // Motor period in us from response
volatile float bdshot600_dma_motor_frequency[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                                   // Motor frequency from response
volatile float bdshot600_dma_motor_rpm[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                                         // Motor rpm from response
volatile uint8_t bdshot600_dma_motor_list_size = 0;                                                                                             // How many motors are initialized in the array
volatile uint8_t bdshot600_dma_busy_flag = 0;                                                                                                   // To know when data is being written to the clock cycle array
volatile uint8_t bdshot600_dma_conversion_finished = 0;                                                                                         // To know if the data conversion has been already completed and no new data arrived
uint8_t bdshot600_dma_remapped_motor_indexes[BDSHOT600_DMA_AMOUNT_OF_MOTORS];                                                                   // Recalculated motor indexes to use same timers in a row
uint8_t bdshot600_dma_motor_indexes_were_remapped = 0;                                                                                          // Lets the driver know if the optimization of motor calls was performed and if optimized motor indexes are available

uint8_t bdshot600_dma_number_of_rotor_pole_pairs = 0;
float bdshot600_dma_number_of_rotor_pole_pairs_division_precalculated = 0;

uint32_t last_time_throttle_was_set_ms = 0;
uint32_t throttle_set_timeout_ms = 0;



// Reverse GCR mapping table
const uint8_t bdshot600_dma_reverse_gcr_map[32] = {
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

uint32_t timings[20];

// Initialize all the peripherals the motor needs to function
int8_t bdshot600_dma_add_motor(GPIO_TypeDef* motor_port, uint16_t motor_pin, TIM_TypeDef* timer_id, uint32_t timer_channel_id, DMA_Stream_TypeDef *dma_instance_stream, uint32_t dma_channel){

    memset(
        timings, 
        0, 
        20 * sizeof(uint8_t)
    );

    // Check if the motor limit is reached
    if (bdshot600_dma_motor_list_size >= BDSHOT600_DMA_AMOUNT_OF_MOTORS) {
        printf("bdshot600_dma - Max number of motors reached\n");
        return -1;
    }

    // Save some data for the motor
    bdshot600_dma_motor_ports[bdshot600_dma_motor_list_size] = motor_port;
    bdshot600_dma_motor_pins[bdshot600_dma_motor_list_size] = motor_pin;
    bdshot600_dma_timer_id[bdshot600_dma_motor_list_size] = timer_id;
    bdshot600_dma_timer_channel_id[bdshot600_dma_motor_list_size] = timer_channel_id;

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
        bdshot600_dma_motor_input_pin_alternate_mode[bdshot600_dma_motor_list_size] = GPIO_AF1_TIM1;
    }else if(timer_id == TIM2){
        __HAL_RCC_TIM2_CLK_ENABLE();
        bdshot600_dma_motor_input_pin_alternate_mode[bdshot600_dma_motor_list_size] = GPIO_AF1_TIM2;
    }else if(timer_id == TIM3){
        __HAL_RCC_TIM3_CLK_ENABLE();
        bdshot600_dma_motor_input_pin_alternate_mode[bdshot600_dma_motor_list_size] = GPIO_AF2_TIM3;
    }else if(timer_id == TIM4){
        __HAL_RCC_TIM4_CLK_ENABLE();
        bdshot600_dma_motor_input_pin_alternate_mode[bdshot600_dma_motor_list_size] = GPIO_AF2_TIM4;
    }else if(timer_id == TIM5){
        __HAL_RCC_TIM5_CLK_ENABLE();
        bdshot600_dma_motor_input_pin_alternate_mode[bdshot600_dma_motor_list_size] = GPIO_AF2_TIM5;
    }

    // Set timer channel flag to check
    if(timer_channel_id == TIM_CHANNEL_1) bdshot600_dma_timer_channel_flag[bdshot600_dma_motor_list_size] = TIM_FLAG_CC1;
    else if(timer_channel_id == TIM_CHANNEL_2) bdshot600_dma_timer_channel_flag[bdshot600_dma_motor_list_size] = TIM_FLAG_CC2;
    else if(timer_channel_id == TIM_CHANNEL_3) bdshot600_dma_timer_channel_flag[bdshot600_dma_motor_list_size] = TIM_FLAG_CC3;
    else if(timer_channel_id == TIM_CHANNEL_4) bdshot600_dma_timer_channel_flag[bdshot600_dma_motor_list_size] = TIM_FLAG_CC4;

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
    // set the motor high as this is normal bdshot600_dma. It is high by default
    HAL_GPIO_WritePin(motor_port, motor_pin, 1);

    //Configure the timer
    

    bdshot600_dma_timer[bdshot600_dma_motor_list_size].Instance = timer_id;
    bdshot600_dma_timer[bdshot600_dma_motor_list_size].Init.Prescaler = 0;  // To divide the clock frequency and get accurate timing
    bdshot600_dma_timer[bdshot600_dma_motor_list_size].Init.CounterMode = TIM_COUNTERMODE_UP;
    bdshot600_dma_timer[bdshot600_dma_motor_list_size].Init.Period = 9300-1;  // Maximum count to measure long pulses
    bdshot600_dma_timer[bdshot600_dma_motor_list_size].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    bdshot600_dma_timer[bdshot600_dma_motor_list_size].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if((ret = HAL_TIM_IC_Init(&bdshot600_dma_timer[bdshot600_dma_motor_list_size])) != HAL_OK){
        printf("bdshot600_dma_add_motor - error, failed to initialize timer: %02x\n", ret);
        return -1;
    }

    bdshot600_dma_timer_channel[bdshot600_dma_motor_list_size].ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;  // Rising edge to start
    bdshot600_dma_timer_channel[bdshot600_dma_motor_list_size].ICSelection = TIM_ICSELECTION_DIRECTTI;
    bdshot600_dma_timer_channel[bdshot600_dma_motor_list_size].ICPrescaler = TIM_ICPSC_DIV1;
    bdshot600_dma_timer_channel[bdshot600_dma_motor_list_size].ICFilter = 0;
    if((ret = HAL_TIM_IC_ConfigChannel(&bdshot600_dma_timer[bdshot600_dma_motor_list_size], &bdshot600_dma_timer_channel[bdshot600_dma_motor_list_size], timer_channel_id)) != HAL_OK){
        printf("bdshot600_dma_add_motor - error, failed to initialize timer channel: %02x\n", ret);
        return -1;
    }

    // Only timer 1 uses DMA2
    if(timer_id == TIM1) __HAL_RCC_DMA2_CLK_ENABLE();
    else __HAL_RCC_DMA1_CLK_ENABLE();

    bdshot600_dma_handle[bdshot600_dma_motor_list_size].Instance = dma_instance_stream;
    bdshot600_dma_handle[bdshot600_dma_motor_list_size].Init.Channel = dma_channel;
    bdshot600_dma_handle[bdshot600_dma_motor_list_size].Init.Direction = DMA_PERIPH_TO_MEMORY;
    bdshot600_dma_handle[bdshot600_dma_motor_list_size].Init.PeriphInc = DMA_PINC_DISABLE;
    bdshot600_dma_handle[bdshot600_dma_motor_list_size].Init.MemInc = DMA_MINC_ENABLE;
    bdshot600_dma_handle[bdshot600_dma_motor_list_size].Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    bdshot600_dma_handle[bdshot600_dma_motor_list_size].Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    bdshot600_dma_handle[bdshot600_dma_motor_list_size].Init.Mode = DMA_NORMAL;
    bdshot600_dma_handle[bdshot600_dma_motor_list_size].Init.Priority = DMA_PRIORITY_VERY_HIGH;
    bdshot600_dma_handle[bdshot600_dma_motor_list_size].Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if ((ret = HAL_DMA_Init(&bdshot600_dma_handle[bdshot600_dma_motor_list_size])) != HAL_OK) {
        printf("bdshot600_dma - DMA initialization error: %d\n", ret);
        return -1;
    }

    // Link DMA handle to timer handle
    // Use the appropriate DMA ID based on the timer channel
    if(bdshot600_dma_timer_channel_id[bdshot600_dma_motor_list_size] == TIM_CHANNEL_1) __HAL_LINKDMA(&bdshot600_dma_timer[bdshot600_dma_motor_list_size], hdma[TIM_DMA_ID_CC1], bdshot600_dma_handle[bdshot600_dma_motor_list_size]);
    else if(bdshot600_dma_timer_channel_id[bdshot600_dma_motor_list_size] == TIM_CHANNEL_2) __HAL_LINKDMA(&bdshot600_dma_timer[bdshot600_dma_motor_list_size], hdma[TIM_DMA_ID_CC2], bdshot600_dma_handle[bdshot600_dma_motor_list_size]);
    else if(bdshot600_dma_timer_channel_id[bdshot600_dma_motor_list_size] == TIM_CHANNEL_3) __HAL_LINKDMA(&bdshot600_dma_timer[bdshot600_dma_motor_list_size], hdma[TIM_DMA_ID_CC3], bdshot600_dma_handle[bdshot600_dma_motor_list_size]);
    else if(bdshot600_dma_timer_channel_id[bdshot600_dma_motor_list_size] == TIM_CHANNEL_4) __HAL_LINKDMA(&bdshot600_dma_timer[bdshot600_dma_motor_list_size], hdma[TIM_DMA_ID_CC4], bdshot600_dma_handle[bdshot600_dma_motor_list_size]);

    // Get the correct interrupt
    IRQn_Type irq_type;
    if(dma_instance_stream == DMA1_Stream0) irq_type = DMA1_Stream0_IRQn;
    else if(dma_instance_stream == DMA1_Stream1) irq_type = DMA1_Stream1_IRQn;
    else if(dma_instance_stream == DMA1_Stream2) irq_type = DMA1_Stream2_IRQn;
    else if(dma_instance_stream == DMA1_Stream3) irq_type = DMA1_Stream3_IRQn;
    else if(dma_instance_stream == DMA1_Stream4) irq_type = DMA1_Stream4_IRQn;
    else if(dma_instance_stream == DMA1_Stream5) irq_type = DMA1_Stream5_IRQn;
    else if(dma_instance_stream == DMA1_Stream6) irq_type = DMA1_Stream6_IRQn;
    else if(dma_instance_stream == DMA1_Stream7) irq_type = DMA1_Stream7_IRQn;
    else if(dma_instance_stream == DMA2_Stream0) irq_type = DMA2_Stream0_IRQn;
    else if(dma_instance_stream == DMA2_Stream1) irq_type = DMA2_Stream1_IRQn;
    else if(dma_instance_stream == DMA2_Stream2) irq_type = DMA2_Stream2_IRQn;
    else if(dma_instance_stream == DMA2_Stream3) irq_type = DMA2_Stream3_IRQn;
    else if(dma_instance_stream == DMA2_Stream4) irq_type = DMA2_Stream4_IRQn;
    else if(dma_instance_stream == DMA2_Stream5) irq_type = DMA2_Stream5_IRQn;
    else if(dma_instance_stream == DMA2_Stream6) irq_type = DMA2_Stream6_IRQn;
    else if(dma_instance_stream == DMA2_Stream7) irq_type = DMA2_Stream7_IRQn;
    else return -1; // Should not get this far

    HAL_NVIC_SetPriority(irq_type, 0, 0);
    HAL_NVIC_EnableIRQ(irq_type);
    
    //get the status flags 
    bdshot600_dma_motor_dma_status[bdshot600_dma_motor_list_size] = 0;

    bdshot600_dma_motor_list_size++;
    return bdshot600_dma_motor_list_size-1;
}

uint8_t  bdshot600_dma_set_rotor_poles_amount(uint8_t rotor_poles){
    // check if number is odd and if it is quit
    if(rotor_poles % 2){
        printf("bdshot600_set_rotor_poles_amount - rotor poles amount is odd, it should be even");
        return 0;
    }
    bdshot600_dma_number_of_rotor_pole_pairs = rotor_poles / 2; // Pairs so we half them

    bdshot600_dma_number_of_rotor_pole_pairs_division_precalculated = 1.0f / bdshot600_dma_number_of_rotor_pole_pairs;

    return 1;
}

// Safety feature. If a throttle value is not updated for this amount of miliseconds the motors will stop getting data. Useful in case the main loop lags really hard and there is an interrupt that wants to keep the motors spinning.
void bdshot600_dma_set_motor_timeout(uint16_t miliseconds){
    throttle_set_timeout_ms = miliseconds;
}

// Call this after all motors are added. Optimizes the usage of the motor pins to use the same timer multiple times in a row
void bdshot600_dma_optimize_motor_calls(){
    uint8_t new_list_index = 0;

    uint8_t amount_of_possible_timers = 5;
    TIM_TypeDef* possible_timers[] = {TIM1, TIM2, TIM3, TIM4, TIM5};
    
    // Find all TIM1 motors and put them in the list first
    for (size_t i = 0; i < amount_of_possible_timers; i++){
        uint8_t amount_of_motors_in_this_timer = 0;

        for (uint8_t k = 0; k < bdshot600_dma_motor_list_size; k++){
            // Iterate over motors and check which ones use this timer
            if(bdshot600_dma_timer_id[k] == possible_timers[i]){
                // Map the index of this motor to a new value of index
                bdshot600_dma_remapped_motor_indexes[k] = new_list_index;
                new_list_index++;
                // If it is the first motor for this timer then configure it to reset the timer when it is sending data
                // Otherwise set it as it does not reset the timer
                if(amount_of_motors_in_this_timer == 0) bdshot600_dma_timer_reset[k] = 1;
                else bdshot600_dma_timer_reset[k] = 0;

                // Count how many motors have been added on behalf of this timer
                amount_of_motors_in_this_timer++;
            }
        }

        if(amount_of_motors_in_this_timer == 0) continue;

        uint16_t new_period_value = 5100 + (amount_of_motors_in_this_timer * 4200);
        // 1 motor will be 5100 
        // 2 motors will be 9300
        // ...
        
        // Use the new indexes to apply the changes to these motors that use this timer
        for (uint8_t k = new_list_index-amount_of_motors_in_this_timer; k < new_list_index; k++){
            // Set how long the timer period should be after knowing how many motors use this timer
            __HAL_TIM_SET_AUTORELOAD(&bdshot600_dma_timer[bdshot600_dma_remapped_motor_indexes[k]], new_period_value-1);  // Set new period
            __HAL_TIM_SET_COUNTER(&bdshot600_dma_timer[bdshot600_dma_remapped_motor_indexes[k]], 0);                      // Reset counter
            bdshot600_dma_timer[bdshot600_dma_remapped_motor_indexes[k]].Instance->EGR = TIM_EGR_UG;                      // Force update of registers
        }
    }

    // Debug
    // printf("bdshot600_dma - new motor index mappings\n");
    // for (uint8_t i = 0; i < bdshot600_dma_motor_list_size; i++){
    //     printf("motor %d -> %d\n", i, bdshot600_dma_remapped_motor_indexes[i]);
    // }
    

    bdshot600_dma_motor_indexes_were_remapped = 1;
}

// Computes the packet for sending it to motor. Can be used for commands also
void bdshot600_dma_set_throttle(uint16_t throttle, uint8_t motor_index){
    uint16_t * motor_packet_pointer = &bdshot600_dma_motor_data_packet[motor_index];

    // Reset the value
    *motor_packet_pointer = 0;

    // Put the throttle info into the packet. Shift it for the telemetry bit
    *motor_packet_pointer = throttle << 1; 
    if(throttle < 48 && throttle > 0) *motor_packet_pointer |= 1;

    // Calculate the 4 bit crc and then invert it, then add it to the packet
    uint8_t crc = (~(*motor_packet_pointer ^ (*motor_packet_pointer >> 4) ^ (*motor_packet_pointer >> 8))) & 0x0F;
    *motor_packet_pointer = (*motor_packet_pointer << 4) | crc;

    last_time_throttle_was_set_ms = HAL_GetTick();
}

// Send throttle that was set to all motors
// Has to be called at 500Hz or more otherwise the motor wont do anything
void bdshot600_dma_send_all_motor_data(){

    // Safety check for stale throttle data
    if(throttle_set_timeout_ms != 0 && HAL_GetTick() - last_time_throttle_was_set_ms > throttle_set_timeout_ms){
        return;
    }

    // Set as busy so reading from the data arrays does not happen by the main loop
    bdshot600_dma_busy_flag = 1;
    bdshot600_dma_conversion_finished = 0;
    for (uint8_t i = 0; i < bdshot600_dma_motor_list_size; i++){
        uint8_t motor_index = bdshot600_dma_motor_indexes_were_remapped * bdshot600_dma_remapped_motor_indexes[i] + i - (bdshot600_dma_motor_indexes_were_remapped * i);

        bdshot600_dma_send_command(motor_index);
    }
    bdshot600_dma_busy_flag = 0;
}

// Check if the driver is busy sending and getting data from motors
uint8_t bdshot600_dma_get_busy(){
    return bdshot600_dma_busy_flag;
}

// Replacement function as the HAL one is broken
HAL_StatusTypeDef new_HAL_TIM_IC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel){
    HAL_StatusTypeDef status = HAL_OK;

    /* Check the parameters */
    assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));
    assert_param(IS_TIM_DMA_CC_INSTANCE(htim->Instance));

    /* Disable the Input Capture channel */
    TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_DISABLE);

    switch (Channel){
        case TIM_CHANNEL_1:
            /* Disable the TIM Capture/Compare 1 DMA request */
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
            (void)HAL_DMA_Abort(htim->hdma[TIM_DMA_ID_CC1]);
            break;
        case TIM_CHANNEL_2:
            /* Disable the TIM Capture/Compare 2 DMA request */
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
            (void)HAL_DMA_Abort(htim->hdma[TIM_DMA_ID_CC2]);
            break;
        case TIM_CHANNEL_3:
            /* Disable the TIM Capture/Compare 3  DMA request */
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
            (void)HAL_DMA_Abort(htim->hdma[TIM_DMA_ID_CC3]);
            break;
        case TIM_CHANNEL_4:
            /* Disable the TIM Capture/Compare 4  DMA request */
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
            (void)HAL_DMA_Abort(htim->hdma[TIM_DMA_ID_CC4]);
            break;
        
        default:
            status = HAL_ERROR;
            break;
    }

    if (status == HAL_OK){
        /* Disable the Peripheral */
        __HAL_TIM_DISABLE(htim);

        /* Set the TIM channel state */
        TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_READY);
        TIM_CHANNEL_N_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_READY);
    }

    /* Return function status */
    return status;
}

void bdshot600_dma_send_command(uint8_t motor_index){
    // Declare all the variables before starting time sensitive stuff
    uint16_t packet = bdshot600_dma_motor_data_packet[motor_index];
    uint32_t pin_mask = bdshot600_dma_motor_pins[motor_index];
    GPIO_TypeDef* port = bdshot600_dma_motor_ports[motor_index];
    uint32_t gpio_bsrr_addr = (uint32_t)&(port->BSRR); // Use the register of the port 
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_HandleTypeDef* timer = &bdshot600_dma_timer[motor_index];
    uint32_t timer_channel = bdshot600_dma_timer_channel_id[motor_index];
    DMA_HandleTypeDef* dma = &bdshot600_dma_handle[motor_index];
    uint16_t * motor_response_clock_cycles_dma = bdshot600_dma_motor_response_clock_cycles_dma[motor_index];
    uint8_t pin_alternate = bdshot600_dma_motor_input_pin_alternate_mode[motor_index];
    uint8_t timer_reset = bdshot600_dma_timer_reset[motor_index];

    // Stop the dma if it did not complete 
    if(bdshot600_dma_motor_dma_status[motor_index] == 1 && __HAL_DMA_GET_TC_FLAG_INDEX(dma) != SET){
        new_HAL_TIM_IC_Stop_DMA(timer, timer_channel);
    }
    // Has to be stopped at this point
    bdshot600_dma_motor_dma_status[motor_index] = 0;

    // Set as output
    GPIO_InitStruct.Pin = pin_mask;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(port, pin_mask, 1);

    // Set all array indexes to 0
    memset(
        motor_response_clock_cycles_dma,
        0,
        BDSHOT600_DMA_CAPTURE_COMPARE_CYCLES_SIZE_DMA * sizeof(uint16_t)
    );

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
        ".rept 110                    \n"
        "NOP                          \n"
        ".endr                        \n"
        "STR r1, [r0]                 \n" // Set pin high
        // How many NOP for bit 1 while it is HIGH
        ".rept 36                     \n"
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
        ".rept 100                    \n"
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
    GPIO_InitStruct.Pin = pin_mask;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = pin_alternate;
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    // Reset the counter and everything about the timer. Only if needed for this motor
    if(timer_reset){
        __HAL_TIM_DISABLE(timer); // Disable timer just in case
        __HAL_TIM_SET_COUNTER(timer, 0); // Reset the timer counter to zero
        __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE); // Clear counter overflow flag

        // Clear capture compare interrupt flag
        if(timer_channel == TIM_CHANNEL_1) __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_CC1);
        else if(timer_channel == TIM_CHANNEL_2) __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_CC2);
        else if(timer_channel == TIM_CHANNEL_3) __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_CC3);
        else if(timer_channel == TIM_CHANNEL_4) __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_CC4);
    }

    // Start the dma timer
    HAL_StatusTypeDef start_response = HAL_TIM_IC_Start_DMA(
        timer,
        timer_channel,
        motor_response_clock_cycles_dma,
        BDSHOT600_DMA_CAPTURE_COMPARE_CYCLES_SIZE_DMA
    );
    
    if (start_response != HAL_OK) {
        // printf("bdshot600_dma dma failed to start, motor-%d, code-%d\n", motor_index, start_response);
        bdshot600_dma_motor_dma_status[motor_index] = 0;
    }else{
        bdshot600_dma_motor_dma_status[motor_index] = 1;
        // printf("Ok%d\n", motor_index);
    }

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
uint8_t bdshot600_dma_convert_all_responses(uint8_t wait_for_data){
    timings[0] = DWT->CYCCNT;
    uint8_t error = 0;
    // Check if this data has already been converted
    if(bdshot600_dma_conversion_finished) return 1;

    // Get the data to then other array
    timings[2] = DWT->CYCCNT;
    for (uint8_t i = 0; i < bdshot600_dma_motor_list_size; i++){ 
        // branchless
        uint8_t motor_index = bdshot600_dma_motor_indexes_were_remapped * bdshot600_dma_remapped_motor_indexes[i] + i - (bdshot600_dma_motor_indexes_were_remapped * i);

        // Wait for the timer overflow flag as that tell us that the timers job is finished
        while(wait_for_data && !(bdshot600_dma_timer[motor_index].Instance->SR & TIM_FLAG_UPDATE));

        // Copy the data as fast as possible to the other array
        memcpy(
            bdshot600_dma_motor_response_clock_cycles[motor_index], 
            bdshot600_dma_motor_response_clock_cycles_dma[motor_index], 
            BDSHOT600_DMA_CAPTURE_COMPARE_CYCLES_SIZE_DMA * sizeof(uint16_t)
        );
    }
    timings[3] = DWT->CYCCNT;


    // Convert the data
    for (uint8_t i = 0; i < bdshot600_dma_motor_list_size; i++){ 
        uint8_t motor_index = bdshot600_dma_motor_indexes_were_remapped * bdshot600_dma_remapped_motor_indexes[i] + i - (bdshot600_dma_motor_indexes_were_remapped * i);

        // Convert the data into actual input capture compare and then try to to convert the data
        if (!bdshot600_dma_prepare_response(motor_index) || !bdshot600_dma_convert_response_to_data(motor_index)) error = 1;
    }
    timings[4] = DWT->CYCCNT;

    // Debug
    // for (uint8_t i = 0; i < bdshot600_dma_motor_list_size; i++){
    //     printf("Motor %d\n", i);
    //     for (uint8_t k = 0; k < BDSHOT600_DMA_CAPTURE_COMPARE_CYCLES_SIZE_DMA; k++) {
    //         printf("%d ", bdshot600_dma_motor_response_clock_cycles[i][k]);
    //     }
    //     printf("\n");
    // }
    

    // Error return here so i can print something out in debug
    if(error) return 0;

    bdshot600_dma_conversion_finished = 1;
    return 1;
}

// Run before convert to data. This will convert the dma responses to normal responses as delta clock cycles
uint8_t  bdshot600_dma_prepare_response(uint8_t motor_index){
    timings[5] = DWT->CYCCNT;
    uint16_t * motor_response_clock_cycles = bdshot600_dma_motor_response_clock_cycles[motor_index];

    uint8_t values_modified = 0;
    // Iterate over all array indexes except last one
    for(uint8_t i = 0; i < BDSHOT600_DMA_CAPTURE_COMPARE_CYCLES_SIZE-1; i++) {
        
        // if no data is set on the next index then quit
        if(motor_response_clock_cycles[i+1] == 0) break;

        // Check if the next value is lower than this one. Indicates that the timer counter overflew.
        if(motor_response_clock_cycles[i+1] - motor_response_clock_cycles[i] < 0 ){
            printf("bdshot600_dma - Timer period needs to be adjusted\n");
            return 0;
        }

        // Delta between these values
        motor_response_clock_cycles[i] = motor_response_clock_cycles[i+1] - motor_response_clock_cycles[i];
        values_modified++;
    }

    // Set remaining to zero
    memset(
        motor_response_clock_cycles + values_modified,
        0,
        BDSHOT600_DMA_CAPTURE_COMPARE_CYCLES_SIZE-values_modified
    );

    timings[6] = DWT->CYCCNT;

    return 1;
}

// This is a separate function form the interrupt to keep the interrupt as short as possible
uint8_t bdshot600_dma_convert_response_to_data(uint8_t motor_index){
    uint16_t* motor_period_us = &bdshot600_dma_motor_period_us[motor_index];
    float* motor_frequency = &bdshot600_dma_motor_frequency[motor_index];
    float* motor_rpm = &bdshot600_dma_motor_rpm[motor_index];
    uint16_t* motor_response_clock_cycles = bdshot600_dma_motor_response_clock_cycles[motor_index];

    timings[7] = DWT->CYCCNT;

    // Check if any clock cycles captured
    if(motor_response_clock_cycles[0] == 0){
        // printf("First index is 0\n");
        *motor_period_us = 0;
        *motor_frequency = 0.0f;
        *motor_rpm = 0.0f;
        return 0;
    }

    // Convert to bits
    uint8_t bit_amount = 0;
    // The first bit will always be a zero and that one is not even shown so first bit is actually 1
    uint8_t bit_value = 0; // Keep track of what bit value we are counting right now
    uint32_t response_in_bits = 0;
    timings[8] = DWT->CYCCNT;


    // Loop over the clock cycles amounts and see how many bits in length each block of these cycles is
    for (uint8_t i = 0; i < BDSHOT600_DMA_CAPTURE_COMPARE_CYCLES_SIZE; i++){
        // If the clock cycle value is 0 at any point, break the loop
        if(motor_response_clock_cycles[i] == 0) break;

        // Rounding to nearest integer is important here. Default integer rounding down is not good.
        timings[15] = DWT->CYCCNT;

        uint16_t number_of_bits_in_this_clock_cycle_capture = (motor_response_clock_cycles[i]+one_bit_cycles_half)/one_bit_cycles;

        timings[16] = DWT->CYCCNT;

        // Place the number of bits in the response
        for (uint16_t j = 0; j < number_of_bits_in_this_clock_cycle_capture; j++){
            // Shift it to the left by one bit
            response_in_bits <<= 1;
            response_in_bits |= bit_value;
            bit_amount++;
        }
        timings[17] = DWT->CYCCNT;

        // If this one was 0 then next one will be 1
        bit_value ^= 1; // Flip the LSB from 0 to 1 and vice versa
    }

    timings[9] = DWT->CYCCNT;

    // The edge detection using input capture compare does not capture the very last bits
    // Make a good assumption on them instead
    // Check how many bits are missing from 21 bit total and add the difference using the current bit value
    if(bit_amount < 21){
        for (uint8_t i = 0; i < 21-bit_amount; i++){
            response_in_bits <<= 1;
            response_in_bits |= bit_value;
        }
    }
    timings[10] = DWT->CYCCNT;
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
    uint8_t a_16bit_nibble_1 = bdshot600_dma_reverse_gcr_map[gcr_nibble_1];  // Converted: 0xF -> 1111
    uint8_t a_16bit_nibble_2 = bdshot600_dma_reverse_gcr_map[gcr_nibble_2];  // Converted: 0xF -> 1111
    uint8_t a_16bit_nibble_3 = bdshot600_dma_reverse_gcr_map[gcr_nibble_3];  // Converted: 0xF -> 1111
    uint8_t a_16bit_nibble_4 = bdshot600_dma_reverse_gcr_map[gcr_nibble_4];  // Converted: 0x0 -> 0000


    // Combine the bits to one 16 bit value
    uint16_t payload_16bit = 
        (a_16bit_nibble_1 << 12) | 
        (a_16bit_nibble_2 << 8) | 
        (a_16bit_nibble_3 << 4) | 
        a_16bit_nibble_4;
    timings[11] = DWT->CYCCNT;

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
    timings[12] = DWT->CYCCNT;

    // Compare if the crc's are equal
    // If they are not equal, do not continue
    if(calculated_crc != payload_16bit_only_crc){

        // For debugging 
        printf("Failed CRC \n");
        for (int i = 3; i >= 0; i--) printf("%u", (calculated_crc >> i) & 1); 
        printf(" != ");
        for (int i = 3; i >= 0; i--) printf("%u", (payload_16bit_only_crc >> i) & 1); 
        printf("\n");

        // Return 0 to indicate that something wrong happened
        *motor_period_us = 0;
        *motor_frequency = 0.0f;
        *motor_rpm = 0.0f;
        return 0;
    }
    timings[13] = DWT->CYCCNT;

    // Get the period in us, frequency and rpm
    *motor_period_us = payload_16bit_only_payload << payload_16bit_only_exponent; // payload is 9 bits and the exponent can move it 7 bits to the left making it a 16 bit value at max
    
    // This value occurs when throttle is 0
    if(*motor_period_us == 65408){
        *motor_frequency  = 0;
    }else{
        // frequency has to be divided by motor pole pairs to get the actual mechanical frequency
        *motor_frequency  = (1.0/((float)*motor_period_us * MICROSECONDS_TO_SECONDS)) * bdshot600_dma_number_of_rotor_pole_pairs_division_precalculated; // Multiply by fraction as it is more efficient
    }

    *motor_rpm = 60 * *motor_frequency ;
    timings[14] = DWT->CYCCNT;
    return 1;
}

// Motor period
uint16_t bdshot600_dma_get_period_us(uint8_t motor_index){
    return bdshot600_dma_motor_period_us[motor_index];
}

// Motor frequency
float bdshot600_dma_get_frequency(uint8_t motor_index){
    return bdshot600_dma_motor_frequency[motor_index];
}

// Motor rpm
float bdshot600_dma_get_rpm(uint8_t motor_index){
    return bdshot600_dma_motor_rpm[motor_index];
}

// Motor dma instance. It is essential to make an interrupt handler for the dma's
DMA_HandleTypeDef* bdshot600_dma_get_dma_handle(uint8_t motor_index){
    return &bdshot600_dma_handle[motor_index];
}

uint32_t * bdshot600_dma_get_timings(){
    return timings;
}


// ----------------------------------------------------------------------------------------- Example use of driver
// float motor_frequency[] = {0.0, 0.0, 0.0, 0.0};
// float motor_rpm[] = {0.0, 0.0, 0.0, 0.0};

// uint8_t motor_BL;
// uint8_t motor_BR;
// uint8_t motor_FR;
// uint8_t motor_FL;

// Optional 
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
//     if(htim->Instance == TIM3){
//         bdshot600_dma_send_all_motor_data();
//     }
// }

// void DMA2_Stream1_IRQHandler(void){
//     HAL_DMA_IRQHandler(bdshot600_dma_get_dma_handle(motor_BL));
// }

// void DMA2_Stream4_IRQHandler(void){
//     HAL_DMA_IRQHandler(bdshot600_dma_get_dma_handle(motor_BR));
// }

// void DMA1_Stream6_IRQHandler(void){
//     HAL_DMA_IRQHandler(bdshot600_dma_get_dma_handle(motor_FL));
// }

// void DMA1_Stream1_IRQHandler(void){
//     HAL_DMA_IRQHandler(bdshot600_dma_get_dma_handle(motor_FR));
// }

// int main(void){
//     motor_FL = bdshot600_dma_add_motor(GPIOA, GPIO_PIN_1,  TIM2, TIM_CHANNEL_2, DMA1_Stream6, DMA_CHANNEL_3);
//     motor_FR = bdshot600_dma_add_motor(GPIOB, GPIO_PIN_10, TIM2, TIM_CHANNEL_3, DMA1_Stream1, DMA_CHANNEL_3);
//     motor_BL = bdshot600_dma_add_motor(GPIOA, GPIO_PIN_8,  TIM1, TIM_CHANNEL_1, DMA2_Stream1, DMA_CHANNEL_6);
//     motor_BR = bdshot600_dma_add_motor(GPIOA, GPIO_PIN_11, TIM1, TIM_CHANNEL_4, DMA2_Stream4, DMA_CHANNEL_6);

//     bdshot600_dma_optimize_motor_calls();
    
//     while(1){
//         if(bdshot600_dma_convert_all_responses(1)){

//             // Motor 0 - BL, 1 - BR, 2 - FR, 3 - FL
//             motor_frequency[0] = bdshot600_dma_get_frequency(motor_BL);
//             motor_frequency[1] = bdshot600_dma_get_frequency(motor_BR);
//             motor_frequency[2] = bdshot600_dma_get_frequency(motor_FR);
//             motor_frequency[3] = bdshot600_dma_get_frequency(motor_FL);

//             motor_rpm[0] = bdshot600_dma_get_rpm(motor_BL); 
//             motor_rpm[1] = bdshot600_dma_get_rpm(motor_BR);
//             motor_rpm[2] = bdshot600_dma_get_rpm(motor_FR);
//             motor_rpm[3] = bdshot600_dma_get_rpm(motor_FL);
//         }

//         bdshot600_dma_set_throttle(100, motor_BL);
//         bdshot600_dma_set_throttle(100, motor_BR);
//         bdshot600_dma_set_throttle(100, motor_FR);
//         bdshot600_dma_set_throttle(100, motor_FL);

//         // Loop must run at least at 500Hz if calling without a timer
//         bdshot600_dma_send_all_motor_data();
//     }

//     return 0;
// }