#include "./time_keeping.h"

// Get the current updated absolute timestamp
uint64_t get_absolute_time(){
    return absolute_microseconds_since_start + __HAL_TIM_GET_COUNTER(&htim4);
}

void handle_pre_loop_start(){
    printf("Looping\n");

    // Capture any radio messages that were sent durring the boot proccess and discard them
    for (uint8_t i = 0; i < 50; i++){
        if(nrf24_data_available(1)){
            nrf24_receive(rx_data);
        }
    }

    // Reset the timer
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    startup_time_microseconds = get_absolute_time();
    entered_loop = 1;

    // Timing for other functionality
    startup_time_microseconds = get_absolute_time();
    ms5611_loop_start_timestamp = get_absolute_time();
    yaw_loop_start_timestamp = get_absolute_time();
}

void handle_loop_start(){
    loop_start_microseconds = __HAL_TIM_GET_COUNTER(&htim4);
    loop_start_miliseconds = HAL_GetTick();
    loop_start_timestamp_microseconds = get_absolute_time();
}


void handle_loop_end(){

    delta_time_without_waiting_from_previous_loop_microseconds = get_absolute_time() - loop_start_timestamp_microseconds;

    loop_iteration++;

    // printf("b%.1f\n", (float)(__HAL_TIM_GET_COUNTER(&htim4) - loop_start_microseconds)/1000.0);
    // printf("%.1f\n", (float)(__HAL_TIM_GET_COUNTER(&htim4) - loop_start_microseconds)/1000.0);
    // printf("%.2f\n", (float)(__HAL_TIM_GET_COUNTER(&htim4) - loop_start_microseconds)/1000.0);

    // If the delta is less than the target keep looping
    while (precalculated_timing_miliseconds > (__HAL_TIM_GET_COUNTER(&htim4) - loop_start_microseconds));

    // Get the final count and add it to the total microsecond
    // If the timer overflew use the ticks instead

    // Right now this HAL_GetTick method is broken as the HAL_GetTick returns a totaly innacurate value
    // TICK 1961427072.000000 = 1762787968.000000 + ((228252.000000 - 29613.000000) * 1000)

    if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE)){
        // float temp_absl = (float)absolute_microseconds_since_start;
        absolute_microseconds_since_start = absolute_microseconds_since_start + 65536;
        // printf("TICK %f = %f + 65536\n", (float)absolute_microseconds_since_start, temp_absl);
        
        // The bellow is currently not working.

        // absolute_microseconds_since_start = absolute_microseconds_since_start + ((HAL_GetTick() - loop_start_miliseconds) * 1000);
        // printf("TICK %f = %f + ((%f - %f) * 1000)\n", (float)absolute_microseconds_since_start, temp_absl, (float)HAL_GetTick(), (float)loop_start_miliseconds);
        // printf("a%lu\n", HAL_GetTick() - loop_start_miliseconds);

    }else{
        //
        // float temp_absl = (float)absolute_microseconds_since_start;
        absolute_microseconds_since_start = absolute_microseconds_since_start + (__HAL_TIM_GET_COUNTER(&htim4) - loop_start_microseconds);
        // printf("NORM %f = %f + (%f - %f)\n", (float)absolute_microseconds_since_start, temp_absl, (float)__HAL_TIM_GET_COUNTER(&htim4), (float)loop_start_microseconds);
        // printf("a%.1f\n", (float)(__HAL_TIM_GET_COUNTER(&htim4) - loop_start_microseconds)/1000.0);
    }

    // Reset the counter
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    // printf("end %lu\n", absolute_microseconds_since_start);

    delta_time_total_loop_time_previous_loop_microseconds = get_absolute_time() - loop_start_timestamp_microseconds;
}