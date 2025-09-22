
#include "./radio.h"

void handle_radio_communication(){
    __disable_irq();
    if(!nrf24_data_available(1)){
        __enable_irq();
        return;
    }

    nrf24_receive(rx_data);
    __enable_irq();


    uint8_t rx_data_size = strlen(rx_data);

    rx_data_size = 32;
    // printf("Received: '");
    // for (size_t i = 0; i < 32; i++)
    // {
    //     printf("%c", rx_data[i]);
    // }
    // printf("'\n");
    
    // Get the type of request
    extract_request_type(rx_data, rx_data_size, rx_type);

    radio2 = DWT->CYCCNT;
    if(strcmp(rx_type, "js") == 0){
        // printf("JOYSTICK\n");
        last_signal_timestamp_microseconds = get_absolute_time();
        
        extract_joystick_request_values_float(rx_data, rx_data_size, &throttle, &yaw, &roll, &pitch);

        // Reversed
        roll = (-(roll-50.0))+50.0;

        // For the blackbox log
        remote_control[0] = roll-50.0;
        remote_control[1] = pitch-50.0;
        remote_control[2] = yaw-50.0;
        remote_control[3] = throttle+100.0;

        // the controls were inverse

        // If pitch and roll are neutral then try holding position with gps
        if(pitch == 50.0 && roll == 50.0 && gps_fix_type == 3 && use_gps_hold == 1 && gps_target_set == 0){
            gps_position_hold_enabled = 1;
            // Current gps position is the target now
            target_latitude = gps_latitude;
            target_longitude = gps_longitude;
            gps_target_set = 1;
        }else if (pitch != 50.0 || roll != 50.0 ){
            if(use_gps_reset_count >= use_gps_reset_count_to_deactivate){
                gps_position_hold_enabled = 0;
                gps_target_set = 0;
                // printf("gps unset ");

                // LOGGING
                gps_target_unset_logged = 0;
                if(roll != 50.0){
                    gps_target_unset_cause = 1;
                    gps_target_unset_roll_value = roll;
                    gps_target_unset_pitch_value = 0.0f;
                }else if(pitch != 50.0){
                    gps_target_unset_cause = 0;
                    gps_target_unset_roll_value = 0.0f;
                    gps_target_unset_pitch_value = pitch;
                }

                // printf("gps unset ");
                // if(roll != 50.0) printf("r %.1f\n", roll);
                // else if(pitch != 50.0) printf("p %.1f\n", pitch);

                // printf("GPS STATE RESET, GPS FIX %d\n", gps_fix_type);
                use_gps_reset_count = 0;
            }else{
                use_gps_reset_count++;
            }
        }

        // The remote control itself handles the deadband stuff

        // Roll ##################################################################################################################
        if(roll == 50.0){
            acro_target_roll = 0.0;
            angle_target_roll = 0.0;
        }else {
            acro_target_roll = map_value(roll, 0.0, 100.0, -acro_mode_rate_degrees_per_second, acro_mode_rate_degrees_per_second);
            angle_target_roll = map_value(roll, 0.0, 100.0, -max_roll_attack, max_roll_attack);
        }
        // Pitch ##################################################################################################################
        if(pitch == 50.0){
            acro_target_pitch = 0.0;
            angle_target_pitch = 0.0;
        }else{   
            acro_target_pitch = map_value(pitch, 0.0, 100.0, -acro_mode_rate_degrees_per_second, acro_mode_rate_degrees_per_second);
            angle_target_pitch = map_value(pitch, 0.0, 100.0, -max_pitch_attack, max_pitch_attack);
        }

        // if(flight_mode == 0){
        //     printf("Acro targets: %.2f %.2f %.2f\n", acro_target_roll, acro_target_pitch, acro_target_yaw);
        // }else if(flight_mode == 1){
        //     printf("Angle targets: %.2f %.2f\n", angle_target_roll, angle_target_pitch);
        // }

        // Yaw ##################################################################################################################
        if(yaw == 50) acro_target_yaw = 0;
        else acro_target_yaw = map_value(yaw, 0.0, 100.0, -max_yaw_attack, max_yaw_attack);

        // Throttle ##################################################################################################################

        // Throttle is handled every loop in post remote function
    }else if(strcmp(rx_type, "pid") == 0){
        printf("Got pid change\n");
        
        float added_proportional = 0;
        float added_integral = 0;
        float added_derivative = 0;
        float added_master_gain = 0;

        extract_pid_request_values(rx_data, strlen(rx_data), &added_proportional, &added_integral, &added_derivative, &added_master_gain);


        if(pid_change_mode == 0){ // Acro mode change
            acro_roll_pitch_gain_p = BASE_ACRO_roll_pitch_GAIN_P + added_proportional;
            acro_roll_pitch_gain_i = BASE_ACRO_roll_pitch_GAIN_I + added_integral;
            acro_roll_pitch_gain_d = BASE_ACRO_roll_pitch_GAIN_D + added_derivative;
            acro_roll_pitch_master_gain = BASE_ACRO_roll_pitch_MASTER_GAIN + added_master_gain;

            added_acro_roll_pitch_gain_p = added_proportional;
            added_acro_roll_pitch_gain_i = added_integral;
            added_acro_roll_pitch_gain_d = added_derivative;
            added_acro_roll_pitch_master_gain = added_master_gain;

            // Configure the roll pid 
            pid_set_proportional_gain(&acro_roll_pid, acro_roll_pitch_gain_p * acro_roll_pitch_master_gain);
            pid_set_integral_gain(&acro_roll_pid, acro_roll_pitch_gain_i * acro_roll_pitch_master_gain);
            pid_set_derivative_gain(&acro_roll_pid, acro_roll_pitch_gain_d * acro_roll_pitch_master_gain);
            pid_reset_integral_sum(&acro_roll_pid);

            // Configure the pitch pid 
            pid_set_proportional_gain(&acro_pitch_pid, acro_roll_pitch_gain_p * acro_roll_pitch_master_gain);
            pid_set_integral_gain(&acro_pitch_pid, acro_roll_pitch_gain_i * acro_roll_pitch_master_gain);
            pid_set_derivative_gain(&acro_pitch_pid, acro_roll_pitch_gain_d * acro_roll_pitch_master_gain);
            pid_reset_integral_sum(&acro_pitch_pid);
            printf("Changed PID of acro mode\n");
        }else if(pid_change_mode == 1){ // Angle mode change
            angle_roll_pitch_gain_p = BASE_ANGLE_roll_pitch_GAIN_P + added_proportional;
            angle_roll_pitch_gain_i = BASE_ANGLE_roll_pitch_GAIN_I + added_integral;
            angle_roll_pitch_gain_d = BASE_ANGLE_roll_pitch_GAIN_D + added_derivative;
            angle_roll_pitch_master_gain = BASE_ANGLE_roll_pitch_MASTER_GAIN + added_master_gain;

            added_angle_roll_pitch_gain_p = added_proportional;
            added_angle_roll_pitch_gain_i = added_integral;
            added_angle_roll_pitch_gain_d = added_derivative;
            added_angle_roll_pitch_master_gain = added_master_gain;

            // Configure the pitch pid 
            pid_set_proportional_gain(&angle_pitch_pid, angle_roll_pitch_gain_p * angle_roll_pitch_master_gain);
            pid_set_integral_gain(&angle_pitch_pid, angle_roll_pitch_gain_i * angle_roll_pitch_master_gain);
            pid_set_derivative_gain(&angle_pitch_pid, angle_roll_pitch_gain_d * angle_roll_pitch_master_gain);
            pid_reset_integral_sum(&angle_pitch_pid);

            // Configure the roll pid 
            pid_set_proportional_gain(&angle_roll_pid, angle_roll_pitch_gain_p * angle_roll_pitch_master_gain);
            pid_set_integral_gain(&angle_roll_pid, angle_roll_pitch_gain_i * angle_roll_pitch_master_gain);
            pid_set_derivative_gain(&angle_roll_pid, angle_roll_pitch_gain_d * angle_roll_pitch_master_gain);
            pid_reset_integral_sum(&angle_roll_pid);
            printf("Changed PID of angle mode\n");
        }else if(pid_change_mode == 2){ // Both depending on flight mode

            if(flight_mode == 1){ // ACRO MODE
                acro_roll_pitch_gain_p = BASE_ACRO_roll_pitch_GAIN_P + added_proportional;
                acro_roll_pitch_gain_i = BASE_ACRO_roll_pitch_GAIN_I + added_integral;
                acro_roll_pitch_gain_d = BASE_ACRO_roll_pitch_GAIN_D + added_derivative;
                acro_roll_pitch_master_gain = BASE_ACRO_roll_pitch_MASTER_GAIN + added_master_gain;

                added_acro_roll_pitch_gain_p = added_proportional;
                added_acro_roll_pitch_gain_i = added_integral;
                added_acro_roll_pitch_gain_d = added_derivative;
                added_acro_roll_pitch_master_gain = added_master_gain;

                // Configure the roll pid 
                pid_set_proportional_gain(&acro_roll_pid, acro_roll_pitch_gain_p * acro_roll_pitch_master_gain);
                pid_set_integral_gain(&acro_roll_pid, acro_roll_pitch_gain_i * acro_roll_pitch_master_gain);
                pid_set_derivative_gain(&acro_roll_pid, acro_roll_pitch_gain_d * acro_roll_pitch_master_gain);
                pid_reset_integral_sum(&acro_roll_pid);

                // Configure the pitch pid 
                pid_set_proportional_gain(&acro_pitch_pid, acro_roll_pitch_gain_p * acro_roll_pitch_master_gain);
                pid_set_integral_gain(&acro_pitch_pid, acro_roll_pitch_gain_i * acro_roll_pitch_master_gain);
                pid_set_derivative_gain(&acro_pitch_pid, acro_roll_pitch_gain_d * acro_roll_pitch_master_gain);
                pid_reset_integral_sum(&acro_pitch_pid);
                printf("Changed PID of acro mode\n");
            }else if(flight_mode == 2){ // ANGLE MODE
                angle_roll_pitch_gain_p = BASE_ANGLE_roll_pitch_GAIN_P + added_proportional;
                angle_roll_pitch_gain_i = BASE_ANGLE_roll_pitch_GAIN_I + added_integral;
                angle_roll_pitch_gain_d = BASE_ANGLE_roll_pitch_GAIN_D + added_derivative;
                angle_roll_pitch_master_gain = BASE_ANGLE_roll_pitch_MASTER_GAIN + added_master_gain;

                added_angle_roll_pitch_gain_p = added_proportional;
                added_angle_roll_pitch_gain_i = added_integral;
                added_angle_roll_pitch_gain_d = added_derivative;
                added_angle_roll_pitch_master_gain = added_master_gain;

                // Configure the pitch pid 
                pid_set_proportional_gain(&angle_pitch_pid, angle_roll_pitch_gain_p * angle_roll_pitch_master_gain);
                pid_set_integral_gain(&angle_pitch_pid, angle_roll_pitch_gain_i * angle_roll_pitch_master_gain);
                pid_set_derivative_gain(&angle_pitch_pid, angle_roll_pitch_gain_d * angle_roll_pitch_master_gain);
                pid_reset_integral_sum(&angle_pitch_pid);

                // Configure the roll pid 
                pid_set_proportional_gain(&angle_roll_pid, angle_roll_pitch_gain_p * angle_roll_pitch_master_gain);
                pid_set_integral_gain(&angle_roll_pid, angle_roll_pitch_gain_i * angle_roll_pitch_master_gain);
                pid_set_derivative_gain(&angle_roll_pid, angle_roll_pitch_gain_d * angle_roll_pitch_master_gain);
                pid_reset_integral_sum(&angle_roll_pid);
                printf("Changed PID of angle mode\n");
            }else if(flight_mode == 3){ // Alitude hold
                altitude_hold_gain_p = BASE_ALTITUDE_HOLD_GAIN_P + added_proportional;
                altitude_hold_gain_i = BASE_ALTITUDE_HOLD_GAIN_I + added_integral;
                altitude_hold_gain_d = BASE_ALTITUDE_HOLD_GAIN_D + added_derivative;
                altitude_hold_master_gain = BASE_ALTITUDE_HOLD_MASTER_GAIN + added_master_gain;

                added_altitude_hold_gain_p = added_proportional;
                added_altitude_hold_gain_i = added_integral;
                added_altitude_hold_gain_d = added_derivative;
                added_altitude_hold_master_gain = added_master_gain;

                // Configure the pitch pid 
                pid_set_proportional_gain(&altitude_hold_pid, altitude_hold_gain_p * altitude_hold_master_gain);
                pid_set_integral_gain(&altitude_hold_pid, altitude_hold_gain_i * altitude_hold_master_gain);
                pid_set_derivative_gain(&altitude_hold_pid, altitude_hold_gain_d * altitude_hold_master_gain);
                pid_reset_integral_sum(&altitude_hold_pid);
                printf("Changed PID of altitude hold\n");
            }else if(flight_mode == 4 || flight_mode == 5){ // GPS hold
                gps_hold_gain_p = BASE_GPS_HOLD_GAIN_P + added_proportional;
                gps_hold_gain_i = BASE_GPS_HOLD_GAIN_I + added_integral;
                gps_hold_gain_d = BASE_GPS_HOLD_GAIN_D + added_derivative;
                gps_hold_master_gain = BASE_GPS_HOLD_MASTER_GAIN + added_master_gain;

                added_gps_hold_gain_p = added_proportional;
                added_gps_hold_gain_i = added_integral;
                added_gps_hold_gain_d = added_derivative;
                added_gps_hold_master_gain = added_master_gain;

                // Configure the pitch pid 
                pid_set_proportional_gain(&gps_latitude_pid, gps_hold_gain_p * gps_hold_master_gain);
                pid_set_integral_gain(&gps_latitude_pid, gps_hold_gain_i * gps_hold_master_gain);
                pid_set_derivative_gain(&gps_latitude_pid, gps_hold_gain_d * gps_hold_master_gain);
                pid_reset_integral_sum(&gps_latitude_pid);

                // Configure the roll pid 
                pid_set_proportional_gain(&gps_longitude_pid, gps_hold_gain_p * gps_hold_master_gain);
                pid_set_integral_gain(&gps_longitude_pid, gps_hold_gain_i * gps_hold_master_gain);
                pid_set_derivative_gain(&gps_longitude_pid, gps_hold_gain_d * gps_hold_master_gain);
                pid_reset_integral_sum(&gps_longitude_pid);
                printf("Changed PID of GPS hold\n");
            }
        }


        // Start new log blackbox log file if one was running.
        if(sd_card_initialized && use_blackbox_logging){
            // Add end of file to the current file
            uint16_t string_length = 0;
            char *betaflight_end_file_string = (char*)betaflight_blackbox_get_end_of_log(&string_length);

            uint8_t* sd_card_buffer = (uint8_t*)sd_card_get_buffer_pointer(1);
            uint16_t sd_card_buffer_index = 0;
            uint16_t betaflight_data_string_index = 0;
            while(sd_card_buffer_index < string_length){
                sd_card_buffer[sd_card_buffer_index] = betaflight_end_file_string[sd_card_buffer_index];
                sd_card_buffer_increment_index();
                sd_card_buffer_index++;
                betaflight_data_string_index++;
            }
            free(betaflight_end_file_string);



            if(use_simple_async){
                sd_special_write_chunk_of_string_data_no_slave_response(sd_card_get_buffer_pointer(1));
                sd_buffer_clear(1);
            }else{
                if(use_blackbox_logging){
                    sd_special_write_chunk_of_byte_data_async(sd_card_get_buffer_pointer(1), string_length);
                    sd_buffer_swap();
                    sd_buffer_clear(1);
                }else{
                    sd_special_write_chunk_of_string_data_async(sd_card_get_buffer_pointer(1));
                    sd_buffer_swap();
                    sd_buffer_clear(1);
                }
            }

            printf("SD logging: Sent file ending\n");

            printf("SD logging: Waiting for writing and slave ready\n");
            sd_card_wait_for_dma_transfer_complete();
            sd_card_wait_for_slave_ready();
            if(blackbox_write_repeat_logs_in_same_file){
                sd_card_wait_for_dma_transfer_complete();
                sd_card_wait_for_slave_ready();

                uint8_t* sd_card_buffer = sd_card_get_buffer_pointer(1);
                uint16_t betaflight_header_length = 0;
                betaflight_blackbox_wrapper_get_header(
                    REFRESH_RATE_HZ,
                    acro_roll_pitch_gain_p * acro_roll_pitch_master_gain,
                    acro_roll_pitch_gain_i * acro_roll_pitch_master_gain,
                    acro_roll_pitch_gain_d * acro_roll_pitch_master_gain,
                    acro_roll_pitch_gain_p * acro_roll_pitch_master_gain,
                    acro_roll_pitch_gain_i * acro_roll_pitch_master_gain,
                    acro_roll_pitch_gain_d * acro_roll_pitch_master_gain,
                    acro_yaw_gain_p,
                    acro_yaw_gain_i,
                    0,
                    angle_roll_pitch_gain_p * angle_roll_pitch_master_gain,
                    angle_roll_pitch_gain_i * angle_roll_pitch_master_gain,
                    angle_roll_pitch_gain_d * angle_roll_pitch_master_gain,
                    altitude_hold_gain_p * altitude_hold_master_gain,
                    altitude_hold_gain_i * altitude_hold_master_gain,
                    altitude_hold_gain_d * altitude_hold_master_gain,
                    gps_hold_gain_p * gps_hold_master_gain,
                    gps_hold_gain_i * gps_hold_master_gain,
                    gps_hold_gain_d * gps_hold_master_gain,
                    filter_gyro_z_yaw_cutoff_frequency,
                    25, // Integral windup
                    21, // Gyro lowpass cutoff
                    20, // Accelerometer lowpass cutoff
                    dshot_refresh_rate,
                    min_dshot600_throttle_value,
                    max_dshot600_throttle_value,
                    actual_min_dshot600_throttle_value,
                    actual_max_dshot600_throttle_value,
                    d_term_filtering_expo,
                    rpm_notch_filter_q_harmonic_1,
                    rpm_notch_filter_min_frequency,
                    d_term_filtering_min_cutoff,
                    d_term_filtering_max_cutoff,
                    d_term_filtering_expo,
                    &betaflight_header_length,
                    sd_card_buffer,
                    10000,
                    blackbox_debug_mode,
                    COMPLEMENTARY_RATIO_MULTIPLYER_FLIGHT
                );
                
                sd_card_buffer_increment_index_by_amount(betaflight_header_length);
                HAL_Delay(1000);
                sd_special_write_chunk_of_byte_data_no_slave_response(sd_card_get_buffer_pointer(1), betaflight_header_length);

                sd_buffer_swap();
                sd_buffer_clear_index(1); // Reset the index

                HAL_Delay(1000); // wait a bit for the logger to write the data into SD
                printf("SD logging: sent blackbox header\n");

                // __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
                // __HAL_TIM_SET_COUNTER(&htim4, 0);
                // startup_time_microseconds = get_absolute_time();
                // loop_start_miliseconds = 0;
                // startup_time_microseconds = 0;
            }else{

                
                // Exit async mode. This will close the current file also
                uint64_t i = 0;
                while(1){
                    // printf("%lu\n", i);
                    HAL_Delay(20);
                    if(sd_special_leave_async_mode()){
                        printf("SD logging: Exit async mode request\n");
                        break;
                    }
                    i++;
                }
                printf("SD logging: Left async mode\n");

                HAL_Delay(7000);
                printf("SD logging: Waited 7 s\n");


                // Initialize new_file
                setup_logging_to_sd(1);
                printf("SD logging: Initialized logging again\n");

                // Reset the loop state
                // loop_iteration = 1;
                // absolute_microseconds_since_start = 0;
                // last_signal_timestamp_microseconds = 1000000;
                // __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
                // __HAL_TIM_SET_COUNTER(&htim4, 0);
                // startup_time_microseconds = get_absolute_time();
                // loop_start_microseconds = 0;
                // loop_start_miliseconds = 0;
                // startup_time_microseconds = 0;
            }

            // Capture any radio messages that were sent durring the boot proccess and discard them
            for (uint8_t i = 0; i < 50; i++){
                if(nrf24_data_available(1)){
                    nrf24_receive(rx_data);
                }
            }

            // Reset the loop state
            // loop_iteration = 1;
            // absolute_microseconds_since_start = 0;
            // loop_start_microseconds = 0;
            // loop_start_miliseconds = 0;

            // startup_time_microseconds = 0;
            // delta_time = 0;
            // time_since_startup_hours = 0;
            // time_since_startup_minutes = 0;
            // time_since_startup_seconds = 0;
            // time_since_startup_ms = 0;
            // time_since_startup_microseconds = 0;

            // // Reset the timer
            // __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
            // __HAL_TIM_SET_COUNTER(&htim4, 0);
            // startup_time_microseconds = get_absolute_time();
            // entered_loop = 1;

            // loop_start_microseconds = __HAL_TIM_GET_COUNTER(&htim4);
            // loop_start_miliseconds = HAL_GetTick();

            // HAL_Delay((uint16_t)(minimum_signal_timing_seconds * 1000));
        }
        

    }else if(strcmp(rx_type, "remoteSyncBase") == 0){
        printf("Got remoteSyncBase\n");
        send_pid_base_info_to_remote();
    }else if(strcmp(rx_type, "remoteSyncAdded") == 0){
        printf("Got remoteSyncAdded\n");
        send_pid_added_info_to_remote();
    }else if(strcmp(rx_type, "accel") == 0){

        float added_x_axis_offset;
        float added_y_axis_offset;

        printf("Accel: '");
        for (size_t i = 0; i < 32; i++){
            printf("%c", rx_data[i]);
        }
        printf("'\n");

        extract_accelerometer_offsets(rx_data, strlen(rx_data), &added_x_axis_offset, &added_y_axis_offset);
        printf("Got offsets %f %f\n", -added_x_axis_offset, -added_y_axis_offset);

        accelerometer_roll_offset = base_accelerometer_roll_offset - added_x_axis_offset;
        accelerometer_pitch_offset = base_accelerometer_pitch_offset -  added_y_axis_offset;

        pid_reset_integral_sum(&angle_roll_pid);
        pid_reset_integral_sum(&angle_pitch_pid);

        calibrate_gyro(); // Recalibrate the gyro as the temperature affects the calibration
        printf("Calibrated gyro\n");
        // Get the initial position again
        get_initial_position();
        printf("Got initial position again\n");

        // Capture any radio messages that were sent durring the calibration proccess and discard them
        for (uint8_t i = 0; i < 50; i++){
            if(nrf24_data_available(1)){
                nrf24_receive(rx_data);
            }
        }
        printf("Got rid of old radio values\n");
        
        // Reset the sesor fusion and the pid accumulation
    }else if(strcmp(rx_type, "fm") == 0){
        printf("Got flight mode\n");

        uint8_t new_flight_mode;

        extract_flight_mode(rx_data, strlen(rx_data), &new_flight_mode);

        printf("Got new flight mode %d\n", new_flight_mode);
        flight_mode = new_flight_mode;

        set_flight_mode(flight_mode);

        find_accelerometer_error(10);

    }

    rx_type[0] = '\0'; // Clear out the string by setting its first char to string terminator
}


void post_remote_control_step(){
    if(target_altitude_barometer_set == 0 && ms5611_reference_set == 1){
        target_altitude_barometer = altitude_barometer;
        target_altitude_barometer_set = 1;
    }
    // Increment the altitude hold target
    if(flight_mode == 3){


        float throttle_deadzone = apply_dead_zone(throttle, 100.0, 0.0, 10);
        
        // Was above or bellow 50.0 in throttle and now is 50.0 - Set the target again to current
        if (throttle_deadzone == 50.0 && (last_throttle_deadzone > 50.0 || last_throttle_deadzone < 50.0)){
            target_altitude_barometer = altitude_barometer;

        // Was bellow or above 50.0 and went immediately to the opposite side of throttle - Need to set the target, so then it is easy to 
        }else if((throttle_deadzone > 50.0 && last_throttle_deadzone < 50.0) || (throttle < 50.0 && last_throttle_deadzone > 50.0)){
            target_altitude_barometer = altitude_barometer;


        // If target not set again then set 
        }else{

            
            // Add a bit of rate of change dependant on the throttle and the refres rate
            float rate_of_change_addition_altitude = (((apply_dead_zone(throttle-50.0f, 50.0f, -50.0f, 5.0f))/50.0f) * altitude_barometer_rate_of_change_max_cm_s) / REFRESH_RATE_HZ;
            target_altitude_barometer += rate_of_change_addition_altitude;
    
            // printf("Alt %.2f Altitude target: %.2f added %f, th %.1f\n", altitude_barometer, target_altitude_barometer, rate_of_change_addition_altitude, throttle);
        }
        
        last_throttle_deadzone = throttle_deadzone;
    }
}


// Extract the values form a slash separated stirng into specific variables for motion control parameters 
void extract_joystick_request_values_uint(char *request, uint8_t request_size, uint8_t *throttle, uint8_t *yaw, uint8_t *roll, uint8_t *pitch){
    // Skip the request type
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;
    char throttle_string[length + 1];
    strncpy(throttle_string, start, length);
    throttle_string[length] = '\0';
    *throttle = atoi(throttle_string);
    //printf("'%s'\n", throttle_string);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char yaw_string[length + 1];
    strncpy(yaw_string, start, length);
    yaw_string[length] = '\0';
    *yaw = atoi(yaw_string);
    //printf("'%s'\n", yaw_string);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char roll_string[length + 1];
    strncpy(roll_string, start, length);
    roll_string[length] = '\0';
    *roll = atoi(roll_string);
    //printf("'%s'\n", roll_string);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char pitch_string[length + 1];
    strncpy(pitch_string, start, length);
    pitch_string[length] = '\0';
    *pitch = atoi(pitch_string);
    //printf("'%s'\n", pitch_string);
}

// Extract the values from a slash-separated string into specific variables for motion control parameters
void extract_joystick_request_values_float(char *request, uint8_t request_size, float *throttle, float *yaw, float *roll, float *pitch) {
    // Skip the request type
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');

    // Parse throttle
    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;
    char throttle_string[length + 1];
    strncpy(throttle_string, start, length);
    throttle_string[length] = '\0';
    *throttle = atof(throttle_string);

    // Parse yaw
    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char yaw_string[length + 1];
    strncpy(yaw_string, start, length);
    yaw_string[length] = '\0';
    *yaw = atof(yaw_string);

    // Parse roll
    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char roll_string[length + 1];
    strncpy(roll_string, start, length);
    roll_string[length] = '\0';
    *roll = atof(roll_string);

    // Parse pitch
    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char pitch_string[length + 1];
    strncpy(pitch_string, start, length);
    pitch_string[length] = '\0';
    *pitch = atof(pitch_string);
}


// Extract the values form a slash separated stirng into specific variables for pid control parameters 
void extract_pid_request_values(char *request, uint8_t request_size, float *added_proportional, float *added_integral, float *added_derivative, float *added_master){
    // Skip the request type
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;
    char added_proportional_string[length + 1];
    strncpy(added_proportional_string, start, length);
    added_proportional_string[length] = '\0';
    *added_proportional = strtod(added_proportional_string, NULL);
    //printf("'%s'\n", added_proportional);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char added_integral_string[length + 1];
    strncpy(added_integral_string, start, length);
    added_integral_string[length] = '\0';
    *added_integral = strtod(added_integral_string, NULL);
    //printf("'%s'\n", added_integral);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char added_derivative_string[length + 1];
    strncpy(added_derivative_string, start, length);
    added_derivative_string[length] = '\0';
    *added_derivative = strtod(added_derivative_string, NULL);
    //printf("'%s'\n", added_derivative);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char added_master_string[length + 1];
    strncpy(added_master_string, start, length);
    added_master_string[length] = '\0';
    *added_master = strtod(added_master_string, NULL);
    //printf("'%s'\n", added_master);
}

void extract_accelerometer_offsets(char *request, uint8_t request_size, float *added_x_axis_offset, float *added_y_axis_offset){
    // Skip the request type
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;
    char added_x_axis_offset_string[length + 1];
    strncpy(added_x_axis_offset_string, start, length);
    added_x_axis_offset_string[length] = '\0';
    *added_x_axis_offset = strtod(added_x_axis_offset_string, NULL);
    //printf("'%s'\n", added_x_axis_offset);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char added_y_axis_offset_string[length + 1];
    strncpy(added_y_axis_offset_string, start, length);
    added_y_axis_offset_string[length] = '\0';
    *added_y_axis_offset = strtod(added_y_axis_offset_string, NULL);
    //printf("'%s'\n", added_y_axis_offset);
}

void extract_flight_mode(char *request, uint8_t request_size, uint8_t *new_flight_mode){
    // Skip the request type
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;
    char new_flight_mode_string[length + 1];
    strncpy(new_flight_mode_string, start, length);
    new_flight_mode_string[length] = '\0';
    *new_flight_mode = strtoul(new_flight_mode_string, NULL, 10);
    //printf("'%s'\n", added_x_axis_offset);
}

// Extract specifically the request type from a slash separated string
void extract_request_type(char *request, uint8_t request_size, char *type_output){
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;

    // You better be sure the length of the output string is big enough
    strncpy(type_output, start, length);
    type_output[length] = '\0';
    // printf("'%s'\n", type_output);
}


// Switch to transmit mode and send out a the added  pid values to the remote
void send_pid_added_info_to_remote(){
    nrf24_tx_mode(tx_address, 10);
    // Delay a bit to avoid problem
    HAL_Delay(30);

    // Make a slash separated value
    char *string = generate_message_pid_values_nrf24(
        added_angle_roll_pitch_gain_p, 
        added_angle_roll_pitch_gain_i, 
        added_angle_roll_pitch_gain_d,
        added_angle_roll_pitch_master_gain
    );
    
    // The remote always receives data as a gibberish with corrupted characters. Sending many of them will mean the remote can reconstruct the message
    // And no crc check did not work, i tried.
    for (size_t i = 0; i < 200; i++)
    {
        if(!nrf24_transmit(string)){
            printf("FAILED\n"); // Very djank
        }
    }
    free(string);
    
    // Switch back to receiver mode
    nrf24_rx_mode(tx_address, 10);
}

// Apply a dead zone to a value 
float apply_dead_zone(float value, float max_value, float min_value, float dead_zone){
    float mid_point = (max_value + min_value) / 2;
    float half_range = (max_value - min_value) / 2;
    float normalized_value = (value - mid_point) / half_range; // this will be -1 at min_value, +1 at max_value

    float dead_zone_normalized = dead_zone / half_range;

    float return_value;

    // remove the deadzone
    if (normalized_value > dead_zone_normalized) {
        return_value = (normalized_value - dead_zone_normalized) / (1.0 - dead_zone_normalized);
    } else if (normalized_value < -dead_zone_normalized) {
        return_value = (normalized_value + dead_zone_normalized) / (1.0 - dead_zone_normalized);
    } else {
        return_value = 0.0;
    }

    return return_value * half_range + mid_point; // scale back to original range
}

char* generate_message_pid_values_nrf24(float base_proportional, float base_integral, float base_derivative, float base_master){
    // calculate the length of the resulting string

    // the s is there for reasons... I just ran out of space on the 32 byte buffer for sending. It was originally supposed to be a full name
    int length = snprintf(
        NULL, 
        0,
        "/s/%.2f/%.2f/%.2f/%.2f/  ", 
        base_proportional, 
        base_integral, 
        base_derivative, 
        base_master
    );
    
    // allocate memory for the string
    char *string = malloc(length + 1); // +1 for the null terminator

    // format the string
    snprintf(
        (char*)string, 
        length + 1, 
        "/s/%.2f/%.2f/%.2f/%.2f/  ", 
        base_proportional, 
        base_integral, 
        base_derivative, 
        base_master
    );

    return string;
}


// Switch to transmit mode and send out a the base pid values to the remote
void send_pid_base_info_to_remote(){
    nrf24_tx_mode(tx_address, 10);
    // Delay a bit to avoid problem
    HAL_Delay(30);

    // Make a slash separated value
    char *string = generate_message_pid_values_nrf24(
        BASE_ANGLE_roll_pitch_GAIN_P, 
        BASE_ANGLE_roll_pitch_GAIN_I, 
        BASE_ANGLE_roll_pitch_GAIN_D,
        BASE_ANGLE_roll_pitch_MASTER_GAIN
    );
    

    // The remote always receives data as a gibberish with corrupted characters. Sending many of them will mean the remote can reconstruct the message
    // And no crc check did not work, i tried.
    for (size_t i = 0; i < 100; i++)
    {
        if(!nrf24_transmit(string)){
            printf("FAILED\n"); // Very djank
        }
    }
    free(string);
    
    // Switch back to receiver mode
    nrf24_rx_mode(tx_address, 10);
}
