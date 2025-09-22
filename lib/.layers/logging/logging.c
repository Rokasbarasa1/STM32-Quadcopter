#include "./logging.h"

void DMA1_Stream7_IRQHandler(void){
    HAL_DMA_IRQHandler(&hdma_spi3_tx);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    if (hspi->Instance == SPI3){
        sd_card_set_dma_transfer_call_status(0); // Set to zero as dma is not happening anymore
    }
}

void handle_logging(){
    delta_time = get_absolute_time() - startup_time_microseconds; // Microseconds
    time_since_startup_hours =                  delta_time * MICROSECONDS_TO_HOURS;
    time_since_startup_minutes =               (delta_time - time_since_startup_hours * HOURS_TO_MICROSECONDS) * MICROSECONDS_TO_MINUTES;
    time_since_startup_seconds =               (delta_time - time_since_startup_hours * HOURS_TO_MICROSECONDS - time_since_startup_minutes * MINUTES_TO_MICROSECONDS) * MICROSECONDS_TO_SECONDS;
    time_since_startup_ms =                    (delta_time - time_since_startup_hours * HOURS_TO_MICROSECONDS - time_since_startup_minutes * MINUTES_TO_MICROSECONDS - time_since_startup_seconds * SECONDS_TO_MICROSECONDS) * MICROSECONDS_TO_MILISECONDS;
    time_since_startup_microseconds =           delta_time - time_since_startup_hours * HOURS_TO_MICROSECONDS - time_since_startup_minutes * MINUTES_TO_MICROSECONDS - time_since_startup_seconds * SECONDS_TO_MICROSECONDS - time_since_startup_ms * MILISECONDS_TO_MICROSECONDS;
    uint32_t time_blackbox = ((time_since_startup_hours * 60 + time_since_startup_minutes * 60) + time_since_startup_seconds) * 1000000 + time_since_startup_ms * 1000 + time_since_startup_microseconds;

    uint8_t skip_write = 0;

    if(sd_card_initialized){
        uint16_t data_size = 0;

        if(use_blackbox_logging){
            uint16_t data_size_data = 0;
            uint8_t* sd_card_buffer = sd_card_get_buffer_pointer(1);

            float angle_mode_targers[] = {angle_target_roll, angle_target_pitch};
            betaflight_blackbox_get_encoded_data_string(
                loop_iteration,
                time_blackbox,
                PID_proportional,
                PID_integral,
                PID_derivative,
                PID_feed_forward,
                remote_control,
                PID_set_points,
                gyro_angular,
                vertical_velocity,
                acceleration_data,
                motor_power,
                magnetometer_data,
                motor_frequency,
                imu_orientation,
                angle_mode_targers,
                altitude_barometer,
                &data_size_data,
                sd_card_buffer,
                blackbox_debug_mode
            );
            
            uint16_t data_size_gps = 0;
            if(got_gps){
                betaflight_blackbox_get_encoded_gps_string(
                    bn357_get_utc_time_raw(),
                    bn357_get_satellites_quantity(),
                    bn357_get_latitude_decimal_format(),
                    bn357_get_longitude_decimal_format(),
                    &data_size_gps, // it will append but not overwrite
                    sd_card_buffer + data_size_data // Offset the buffer, to not overwrite the data
                );
            }

            // printf("I %d\n", data_size_data);
            // printf("G %d\n", data_size_gps);

            data_size = data_size_data + data_size_gps;
            sd_card_buffer_increment_index_by_amount(data_size);
        }else{
            if(txt_logging_mode == 0){
                // TXT based logging
                // Log a bit of data
                // sd_card_append_to_buffer(1, "%02d:%02d:%02d:%03d;", time_since_startup_hours, time_since_startup_minutes, time_since_startup_seconds, time_since_startup_ms);
                // sd_card_append_to_buffer(1, "ACCEL,%.2f,%.2f,%.2f;", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
                // sd_card_append_to_buffer(1, "GYRO,%.2f,%.2f,%.2f;", imu_orientation[0], imu_orientation[1], imu_orientation[2]);
                // sd_card_append_to_buffer(1, "MAG,%.2f,%.2f,%.2f;", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);
                // sd_card_append_to_buffer(1, "MOTOR,1=%.2f,2=%.2f,3=%.2f,4=%.2f;", motor_power[0], motor_power[1], motor_power[2], motor_power[3]);
                // sd_card_append_to_buffer(1, "ERROR,pitch=%.2f,roll=%.2f,yaw=%.2f,altitude=%.2f;", error_angle_pitch, error_angle_roll, error_acro_yaw, error_altitude);
                // sd_card_append_to_buffer(1, "TEMP,%.2f;", temperature_celsius);
                // sd_card_append_to_buffer(1, "ALT %.2f;", altitude_sensor_fusion);
                // sd_card_append_to_buffer(1, "GPS,lon-%f,lat-%f;", bn357_get_longitude_decimal_format(), bn357_get_latitude_decimal_format());
                // sd_card_append_to_buffer(1, "\n");

                // sd_card_append_to_buffer(1, "%6.5f %6.5f %6.5f", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);  
                // sd_card_append_to_buffer(1, "\n");
                
            }else if(txt_logging_mode == 1){ // Log gps target and current position
                if(got_gps){
                    if(txt_logged_header == 0){
                        sd_card_append_to_buffer(1, "target lat;target lon;lat;lon;satellites count;yaw;gyro yaw;mag yaw;mag2 yaw;gps yaw;gps roll;gps pitch;roll;pitch;lat distance to target meters;lon distance to target meters;extra info;\n");
                        txt_logged_header = 1;
                    }

                    sd_card_append_to_buffer(1, "%.1f;%.1f;%.1f;%.1f;%d;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;", 
                        target_latitude * 1000000.0f,            // %f;
                        target_longitude * 1000000.0f,           // %f;
                        gps_latitude * 1000000.0f,               // %f;
                        gps_longitude * 1000000.0f,              // %f;
                        gps_satellites_count,       // %d;
                        imu_orientation[2],         // %.1f;
                        gyro_yaw,
                        magnetometer_yaw,
                        magnetometer_yaw_secondary,
                        gps_yaw,
                        gps_hold_roll_adjustment,   // %.1f;
                        gps_hold_pitch_adjustment,  // %.1f;
                        imu_orientation[0],         // %.1f;
                        imu_orientation[1]          // %.1f;
                    );

                    if(gps_target_unset_logged == 0){
                        // Set the distance to target meters as empty
                        sd_card_append_to_buffer(1, "0.0;0.0;");
                        // Log some reason for something happening
                        if(gps_target_unset_cause == 1){
                            sd_card_append_to_buffer(1, "r%.3f;", 
                                gps_target_unset_roll_value
                            );
                        }else if(gps_target_unset_cause == 0){
                            sd_card_append_to_buffer(1, "p%.3f;", 
                                gps_target_unset_pitch_value
                            );
                        }else if(gps_target_unset_cause == 3){
                            sd_card_append_to_buffer(1, "gps_can_be_used;");
                        }else if(gps_target_unset_cause == 4){
                            sd_card_append_to_buffer(1, "gps_position_hold_enabled;");
                        }else if(gps_target_unset_cause == 2){
                            if(got_gps){
                                sd_card_append_to_buffer(1, "radoffgot;");
                            }else{
                                sd_card_append_to_buffer(1, "radoff;");
                            }
                        }else if(gps_target_unset_cause == 5){
                            sd_card_append_to_buffer(1, "not level;");
                        }
                        
                        gps_target_unset_logged = 1;
                    }else{
                        // Log the distance to target and empty reason for something
                        sd_card_append_to_buffer(1, "%f;%f;;", 
                            lat_distance_to_target_meters,
                            lon_distance_to_target_meters
                        );
                    }

                    sd_card_append_to_buffer(1, "\n", 
                        gps_target_unset_pitch_value
                    );
                }else{
                    sd_card_append_to_buffer(1, "NAN\n");
                }
            }else if(txt_logging_mode == 2){
                if(got_pressure == 1){
                    if(txt_logged_header == 0){
                        sd_card_append_to_buffer(1, "alt barometer;target alt barometer;throttle actual;throttle;P error;I error;D error;\n");
                        txt_logged_header = 1;
                    }
                    if(ms5611_reference_set == 1){
                        sd_card_append_to_buffer(1, "%.2f;%.2f;%.1f;%.1f;%.2f;%.2f;%.2f;\n", 
                            altitude_barometer,
                            target_altitude_barometer,
                            throttle_value,
                            throttle,
                            PID_proportional[3],
                            PID_integral[3],
                            PID_derivative[3]
                        );
                    }else if(ms5611_reference_set == 0){
                        // IF reference not set then dont log the data
                        sd_card_append_to_buffer(1, "0.0;0.0;0.0;0.0;0.0;0.0;0.0;\n");
                    }
                }else{
                    skip_write = 1;
                }
            }else if(txt_logging_mode == 3){
                float raw_magnetometer_data[] = {0, 0, 0};
                float raw_magnetometer_secondary_data[] = {0, 0, 0};

                bmm350_previous_raw_magetometer_readings(raw_magnetometer_data);
                mmc5603_previous_raw_magetometer_readings(raw_magnetometer_secondary_data);
                // qmc5883l_previous_raw_magetometer_readings(raw_magnetometer_data);
                // hmc5883l_previous_raw_magetometer_readings(raw_magnetometer_data);
                // if(ist8310){
                //     ist8310_previous_raw_magetometer_readings(raw_magnetometer_data);
                // }else{
                //     bmm350_previous_raw_magetometer_readings(raw_magnetometer_data);
                // }


                // sd_card_append_to_buffer(1, "%f\t%f\t%f\t\n", 
                //     raw_magnetometer_data[0],
                //     raw_magnetometer_data[1],
                //     raw_magnetometer_data[2]
                // );
                
                sd_card_append_to_buffer(1, "%f\t%f\t%f\t%f\t%f\t%f\t\n", 
                    raw_magnetometer_data[0],
                    raw_magnetometer_data[1],
                    raw_magnetometer_data[2],
                    raw_magnetometer_secondary_data[0],
                    raw_magnetometer_secondary_data[1],
                    raw_magnetometer_secondary_data[2]
                );
            }else if (txt_logging_mode == 4){
                if(txt_logged_header == 0){
                    sd_card_append_to_buffer(1, "time microseconds;gyro_yaw;magnetometer_yaw;magnetometer_yaw_90;magnetometer_yaw_180;magnetometer_yaw_270;magnetometer_yaw_secondary;gps_yaw;magnetometer_ist8310_yaw;roll;pitch;\n");
                    txt_logged_header = 1;
                }
                char abs_time_str[21];
                uint64_to_str(get_absolute_time(), abs_time_str);
                sd_card_append_to_buffer(1, "%s;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;\n",
                    abs_time_str,
                    gyro_yaw,
                    magnetometer_yaw,
                    magnetometer_yaw_90,
                    magnetometer_yaw_180,
                    magnetometer_yaw_270,
                    magnetometer_yaw_secondary,
                    gps_yaw,
                    magnetometer_ist8310_yaw,
                    imu_orientation[0],
                    imu_orientation[1]
                );
            }else if (txt_logging_mode == 5){

                if(txt_logged_header == 0){
                    sd_card_append_to_buffer(1, "time microseconds;time since last loop;\n");
                    txt_logged_header = 1;
                }


                char abs_time_str[21];
                uint64_to_str(delta_time_total_loop_time_previous_loop_microseconds, abs_time_str);

                char delta_loop_time_str[21];
                uint64_to_str(delta_time_without_waiting_from_previous_loop_microseconds, delta_loop_time_str);
                sd_card_append_to_buffer(1, "%s;%s;\n",
                    abs_time_str,
                    delta_loop_time_str
                );
            }else if (txt_logging_mode == 6){

                if(txt_logged_header == 0){
                    sd_card_append_to_buffer(1, "time microseconds;time since last loop;roll;pitch;yaw;old roll;old pitch;old yaw;gyro yaw;gyro yaw old;mag yaw;mag yaw old;mag x;mag y;mag z;mag raw x;mag raw y;mag raw z;gyro x;gyro y;gyro z;gyro x for derivative;gyro y for derivative;gyro raw x;gyro raw y;gyro raw z;accel x;accel y;accel z;accel raw x;accel raw y;accel raw z;altitude;altitude raw;\n");
                    txt_logged_header = 1;
                }

                char abs_time_str[21];
                uint64_to_str(delta_time_total_loop_time_previous_loop_microseconds, abs_time_str);

                char delta_loop_time_str[21];
                uint64_to_str(delta_time_without_waiting_from_previous_loop_microseconds, delta_loop_time_str);
                sd_card_append_to_buffer(1, "%s;%s;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;\n",
                    abs_time_str,
                    delta_loop_time_str,
                    imu_orientation[0],
                    imu_orientation[1],
                    imu_orientation[2],
                    old_imu_orientation[0],
                    old_imu_orientation[1],
                    old_imu_orientation[2],
                    gyro_yaw,
                    gyro_yaw_old,
                    magnetometer_yaw,
                    0.0f,
                    magnetometer_data[0],
                    magnetometer_data[1],
                    magnetometer_data[2],
                    magnetometer_data_raw[0],
                    magnetometer_data_raw[1],
                    magnetometer_data_raw[2],
                    gyro_angular[0],
                    gyro_angular[1],
                    gyro_angular[2],
                    gyro_angular_for_d_term[0],
                    gyro_angular_for_d_term[1],
                    gyro_angular_raw[0],
                    gyro_angular_raw[1],
                    gyro_angular_raw[2],
                    acceleration_data[0],
                    acceleration_data[1],
                    acceleration_data[2],
                    acceleration_data_raw[0],
                    acceleration_data_raw[1],
                    acceleration_data_raw[2],
                    altitude_barometer,
                    altitude_barometer_raw
                );
            }else if (txt_logging_mode == 7){
                sd_card_append_to_buffer(1, "%f\t%f\t%f\t%f\t%f\t%f\t\n", 
                    magnetometer_data_raw[0],
                    magnetometer_data_raw[1],
                    magnetometer_data_raw[2],
                    magnetometer_data_secondary_raw[0],
                    magnetometer_data_secondary_raw[1],
                    magnetometer_data_secondary_raw[2]
                );
            }else if (txt_logging_mode == 8){

                
                if(txt_logged_header == 0){
                    sd_card_append_to_buffer(1, "motor0 rpm;motor1 rpm;motor2 rpm;motor3 rpm;mag x;mag y;mag z;mag x offset;mag y offset;mag z offset;mag raw x;mag raw y;mag raw z;mag yaw;mag2 x;mag2 y;mag2 z;mag2 x offset;mag2 y offset;mag2 z offset;mag2 raw x;mag2 raw y;mag2 raw z;mag2 yaw;gps yaw;gyro yaw;roll;pitch;avg rpm;\n");
                    txt_logged_header = 1;
                }

                sd_card_append_to_buffer(1, "%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;\n",
                    motor_frequency[0],
                    motor_frequency[1],
                    motor_frequency[2],
                    motor_frequency[3],
                    magnetometer_data[0],
                    magnetometer_data[1],
                    magnetometer_data[2],
                    mag1_x_offset,
                    mag1_y_offset,
                    mag1_z_offset,
                    magnetometer_data_raw[0],
                    magnetometer_data_raw[1],
                    magnetometer_data_raw[2],
                    magnetometer_yaw,
                    magnetometer_data_secondary[0],
                    magnetometer_data_secondary[1],
                    magnetometer_data_secondary[2],
                    mag2_x_offset,
                    mag2_y_offset,
                    mag2_z_offset,
                    magnetometer_data_secondary_raw[0],
                    magnetometer_data_secondary_raw[1],
                    magnetometer_data_secondary_raw[2],
                    magnetometer_yaw_secondary,
                    gps_yaw,
                    gyro_yaw,
                    imu_orientation[0],
                    imu_orientation[1],
                    average_rpm
                );
            }
        }

        // Be sure that the write still happens at least once per second
        if(skip_write) return;

        if(sd_card_async_initialized){
            if(use_simple_async){
                sd_special_write_chunk_of_string_data_no_slave_response(sd_card_get_buffer_pointer(1));
                sd_buffer_clear(1);
            }else{
                if(use_blackbox_logging){
                    logging2 = DWT->CYCCNT;
                    
                    // uint8_t * buffer_new = (uint8_t *)sd_card_get_buffer_pointer(1);
                    // printf("%d write  ", data_size);
                    // for (uint16_t i = 0; i < data_size; i++){
                    //     printf("%02X ", buffer_new[i]);
                    // }
                    // printf("\n");

                    sd_special_write_chunk_of_byte_data_async(sd_card_get_buffer_pointer(1), data_size);

                    sd_buffer_swap();
                    sd_buffer_clear_index(1); // Reset the index
                    // sd_buffer_clear(1); // Dont need to wipe all of it for now

                    logging3 = DWT->CYCCNT;
                }else{
                    
                    // uint8_t* sd_card_buffer = (uint8_t*)sd_card_get_buffer_pointer(1);
                    // printf("%d write:'", strlen(sd_card_buffer));
                    // for (uint16_t i = 0; i < strlen(sd_card_buffer); i++){
                    //     printf("%c", sd_card_buffer[i]);
                    // }
                    // printf("'\n");

                    sd_special_write_chunk_of_string_data_async(sd_card_get_buffer_pointer(1));
                    sd_buffer_swap();
                    sd_buffer_clear_index(1); // Reset the index
                    // sd_buffer_clear(1); // Dont need to wipe all of it for now
                }
            }
        }else{
            if(use_blackbox_logging){
                sd_card_initialized = sd_special_write_chunk_of_byte_data(sd_card_get_buffer_pointer(1), data_size);
            }else{
                sd_card_initialized = sd_special_write_chunk_of_string_data(sd_card_get_buffer_pointer(1));
            }
        }
    }

    // printf("Motor BL: %5.1f BR: %5.1f FR: %5.1f FL: %5.1f \n",
    //     motor_frequency[0],
    //     motor_frequency[1],
    //     motor_frequency[2],
    //     motor_frequency[3]
    // );

    // printf("%5.1f\n",
    //     motor_frequency[0]
    // );

    // printf("ACCEL,%.2f,%.2f,%.2f;", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
    // printf("GYRO,%5.2f,%5.2f,%5.2f;", gyro_angular[0], gyro_angular[1], gyro_angular[2]);
    // printf("MAG,%.2f,%.2f,%.2f;", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);
    // printf("TEMP,%.2f;", temperature_celsius);
    // printf("ALT %.2f;", altitude_barometer);
    // printf("\n");

    // Update the blue led with current sd state
    // GPS stuff more imortant curently so this is disabled
    // if(sd_card_initialized){
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    // }else{
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    // }
}

void setup_logging_to_sd(uint8_t use_updated_file_name){
    // Initialize sd card logging
    sd_card_initialized = 0;
    if(!use_updated_file_name) sd_card_initialize_spi(&hspi3, GPIOA, GPIO_PIN_15, GPIOA, GPIO_PIN_12);
    

    // OK
    if(!sd_test_interface()){
        printf("Failed to initialize the sd card interface: Logging slave issue\n");
        indicate_mistake_on_logger();
        return;
    }

    printf("SD logging: Logger module is working\n");

    // BLACKBOX
    if(use_blackbox_logging){
        if(use_updated_file_name){
            sd_card_initialized = sd_special_initialize(new_log_file_blackbox_base_name);
        }else{
            // OK
            sd_card_initialized = sd_special_initialize(log_file_blackbox_base_name);
        }


        if(!sd_card_initialized){
            printf("Failed to initialize the sd card interface: SD card connection issue\n");
            printf("SD logging: FAILED to initialize blackbox logging. Code %d\n", sd_get_response());
            indicate_mistake_on_sd_card();
            return;
        }

        printf("SD logging: Initialized blackbox logging. Code %d\n", sd_get_response());


        // Only has to be done initially not when new files are opened after that
        if(sd_card_initialized && !use_updated_file_name){
            // Get the index of the file

            // OK
            blackbox_file_index = sd_special_get_file_index();
            printf("SD logging: File index is %d\n", blackbox_file_index);

            // Create the new file name string. Will be used for repeated log starts
            uint16_t length = snprintf(
                NULL, 
                0,
                "-%d%s", 
                blackbox_file_index,
                log_file_blackbox_base_name
            );
            
            // allocate memory for the string
            new_log_file_blackbox_base_name = (uint8_t*)malloc(length + 1); // +1 for the null terminator

            // format the string
            snprintf(
                (char*)new_log_file_blackbox_base_name, 
                length + 1, 
                "-%d%s", 
                blackbox_file_index,
                log_file_blackbox_base_name
            );
        }

        sd_card_async_initialized = sd_special_enter_async_byte_mode(1);// Slave should reset when async stops

        if(!sd_card_async_initialized){
            printf("SD logging: failed to enter async byte mode\n");
            indicate_mistake_on_logger();
            sd_card_initialized = 0;
            return;
        }
        printf("SD logging: entered async BLACKBOX mode\n");

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

        sd_special_write_chunk_of_byte_data_no_slave_response(sd_card_get_buffer_pointer(1), betaflight_header_length);

        sd_buffer_swap();
        sd_buffer_clear_index(1); // Reset the index

        HAL_Delay(1000); // wait a bit for the logger to write the data into SD
        printf("SD logging: sent blackbox header\n");

    // TXT
    }else if(!use_blackbox_logging){
        if(txt_logging_mode == 0) sd_card_initialized = sd_special_initialize(log_file_base_name);
        else if(txt_logging_mode == 1) sd_card_initialized = sd_special_initialize(log_file_base_name_gps);
        else if(txt_logging_mode == 2) sd_card_initialized = sd_special_initialize(log_file_base_name_alt);
        else if(txt_logging_mode == 3) sd_card_initialized = sd_special_initialize(log_file_base_name_mag);
        else if(txt_logging_mode == 4) sd_card_initialized = sd_special_initialize(log_file_base_name_yaw);
        else if(txt_logging_mode == 5) sd_card_initialized = sd_special_initialize(log_file_base_name_timing);
        else if(txt_logging_mode == 6) sd_card_initialized = sd_special_initialize(log_file_base_name_imu);
        else if(txt_logging_mode == 7) sd_card_initialized = sd_special_initialize(log_file_base_name_mag);
        else if(txt_logging_mode == 8) sd_card_initialized = sd_special_initialize(log_file_base_name_compassRPM);
        else sd_card_initialized = sd_special_initialize(log_file_base_name);

        if(!sd_card_initialized){
            printf("SD logging: FAILED to initialize txt logging. Code %d\n", sd_get_response());
            printf("Failed to initialize the sd card interface: SD card connection issue\n");
            indicate_mistake_on_sd_card();
            return;
        }

        printf("SD logging: Initialized txt logging. Code %d\n", sd_get_response());

        sd_card_async_initialized = sd_special_enter_async_string_mode(1);

        if(!sd_card_async_initialized){
            printf("SD logging: failed to enter async TXT mode\n");
            indicate_mistake_on_logger();
            sd_card_initialized = 0;
            return;
        }

        
        sd_buffer_swap();
        sd_buffer_clear_index(1); // Reset the index

        HAL_Delay(1000); // wait a bit for the logger to write the data into SD
        
        printf("SD logging: entered async TXT mode\n");
    }
    
    indicate_sd_logging_ok();
}

void indicate_mistake_on_logger(){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(1000);
}

void indicate_mistake_on_sd_card(){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(200);
}

void indicate_sd_logging_ok(){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(250);
}

void uint64_to_str(uint64_t value, char *buf) {
    char tmp[21];
    int i = 0;
    if (value == 0) {
        buf[0] = '0';
        buf[1] = '\0';
        return;
    }
    while (value > 0) {
        tmp[i++] = '0' + (value % 10);
        value /= 10;
    }
    for (int j = 0; j < i; ++j) {
        buf[j] = tmp[i - j - 1];
    }
    buf[i] = '\0';
}
