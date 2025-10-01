
#include "./motors.h"


// Used by the bdshot600 protocol for all the motors
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
    if(htim->Instance == TIM3){
        interrupt = DWT->CYCCNT;
        bdshot600_dma_send_all_motor_data();
        interrupt_end = DWT->CYCCNT;
    }
}

// Handlers for dma bdshot600 pins
// Without the handlers the DMA will crash the system
void DMA2_Stream1_IRQHandler(void){
    HAL_DMA_IRQHandler(bdshot600_dma_get_dma_handle(motor_BL));
}

void DMA2_Stream4_IRQHandler(void){
    HAL_DMA_IRQHandler(bdshot600_dma_get_dma_handle(motor_BR));
}

void DMA1_Stream6_IRQHandler(void){
    HAL_DMA_IRQHandler(bdshot600_dma_get_dma_handle(motor_FL));
}

void DMA1_Stream1_IRQHandler(void){
    HAL_DMA_IRQHandler(bdshot600_dma_get_dma_handle(motor_FR));
}

void initialize_motor_communication(){
    // The dma streams cannot be conflicting with other peripherals

    motor_FL = bdshot600_dma_add_motor(GPIOA, GPIO_PIN_1,  TIM2, TIM_CHANNEL_2, DMA1_Stream6, DMA_CHANNEL_3);
    motor_FR = bdshot600_dma_add_motor(GPIOB, GPIO_PIN_10, TIM2, TIM_CHANNEL_3, DMA1_Stream1, DMA_CHANNEL_3);
    motor_BL = bdshot600_dma_add_motor(GPIOA, GPIO_PIN_8,  TIM1, TIM_CHANNEL_1, DMA2_Stream1, DMA_CHANNEL_6);
    motor_BR = bdshot600_dma_add_motor(GPIOA, GPIO_PIN_11, TIM1, TIM_CHANNEL_4, DMA2_Stream4, DMA_CHANNEL_6);
    bdshot600_dma_set_rotor_poles_amount(motor_rotor_poles);
    bdshot600_dma_set_motor_timeout(50); // After 50 ms of no setting of throttle, the motor will stop, for safety
    bdshot600_dma_optimize_motor_calls();


    // If if the control loop runs at 500Hz or more then it can handle the motors on its own. Without interrupt
    if(dshot_refresh_rate == REFRESH_RATE_HZ && REFRESH_RATE_HZ >= 500){
        manual_bdshot = 1;
        printf("bdshot600 using manual mode\n");
    }else{
        manual_bdshot = 0;
        HAL_TIM_Base_Start_IT(&htim3); // Start the timer that will be calling the dshot at a set frequneyc
        printf("bdshot600 using interrupt trigger mode\n");
    }
}

void handle_pid_and_motor_control(){    

    // For the robot to do work it needs to be receiving radio signals and at the correct angles, facing up
    if(
        imu_orientation[0] >  max_angle_before_motors_off || 
        imu_orientation[0] < -max_angle_before_motors_off || 
        imu_orientation[1] >  max_angle_before_motors_off || 
        imu_orientation[1] < -max_angle_before_motors_off || 
        ((float)get_absolute_time() - (float)last_signal_timestamp_microseconds) / 1000000.0 > minimum_signal_timing_seconds
    ){  


        motor_off_index++;
        motor_off_index %= REFRESH_RATE_HZ;

        if(motor_off_index == 0){
            // DEBUG Why are motors off?
            if(imu_orientation[0] >  max_angle_before_motors_off){
                printf("MO ROLL+\n");
            }
            if(imu_orientation[0] < -max_angle_before_motors_off){
                printf("MO ROLL-\n");
            }
            if(imu_orientation[1] >  max_angle_before_motors_off){
                printf("MO PITCH+\n");
            }
            if(imu_orientation[1] < -max_angle_before_motors_off){
                printf("MO PITCH-\n");
            }
            if( ((float)get_absolute_time() - (float)last_signal_timestamp_microseconds) / 1000000.0 > minimum_signal_timing_seconds){
                printf("MO RADIO\n");
            }
        }



        // When not receiving signals, increase the complementary filter ratio to quickly smooth out any movements done by carrying the drone
        mpu6050_set_complementary_ratio(COMPLEMENTARY_RATIO_MULTIPLYER_OFF * (1.0 - 1.0/(1.0+(1.0/REFRESH_RATE_HZ))));
        // uint64_t timm_absolute = get_absolute_time();

        // float timessss = ((float)timm_absolute - (float)last_signal_timestamp_microseconds) / 1000000.0;
        // printf("STOP %.1f %.1f - %f %f %f %f %f\n", imu_orientation[0], imu_orientation[1], timessss, (float)timm_absolute, (float)absolute_microseconds_since_start,  (float)absolute_microseconds_since_start, (float)last_signal_timestamp_microseconds);

        // printf("b\n");
        throttle_value_FR = min_dshot600_throttle_value;
        throttle_value_FL = min_dshot600_throttle_value;
        throttle_value_BL = min_dshot600_throttle_value;
        throttle_value_BR = min_dshot600_throttle_value;

        // FPV cam side FRONT
        bdshot600_dma_set_throttle(throttle_value_FR, motor_FR); // Front-right
        bdshot600_dma_set_throttle(throttle_value_FL, motor_FL); // Front-left
        // GPS side BACK
        bdshot600_dma_set_throttle(throttle_value_BL, motor_BL); // Back-left
        bdshot600_dma_set_throttle(throttle_value_BR, motor_BR); // Back-right

        // For logging
        motor_power[0] = throttle_value_BL; // Logging
        motor_power[1] = throttle_value_FL; // Logging
        motor_power[2] = throttle_value_BR; // Logging
        motor_power[3] = throttle_value_FR; // Logging

        // Reset the remote control set points also
        remote_control[0] = 0;
        remote_control[1] = 0;
        remote_control[2] = 0;
        remote_control[3] = 0;

        PID_proportional[0] = 0; // Logging
        PID_proportional[1] = 0; // Logging
        PID_proportional[2] = 0; // Logging

        PID_integral[0] = 0; // Logging
        PID_integral[1] = 0; // Logging
        PID_integral[2] = 0; // Logging

        PID_derivative[0] = 0; // Logging
        PID_derivative[1] = 0; // Logging

        PID_set_points[0] = 0; // Logging
        PID_set_points[1] = 0; // Logging
        PID_set_points[2] = 0; // Logging
        PID_set_points[3] = 0; // Logging

        // gps_hold_roll_adjustment = 0.0f;
        // gps_hold_pitch_adjustment = 0.0f;

        pid_reset_integral_sum(&acro_roll_pid);
        pid_reset_integral_sum(&acro_pitch_pid);
        pid_reset_integral_sum(&acro_yaw_pid);

        pid_reset_integral_sum(&angle_pitch_pid);
        pid_reset_integral_sum(&angle_roll_pid);

        pid_reset_integral_sum(&altitude_hold_pid);
        pid_reset_integral_sum(&altitude_hold_pid);

        pid_reset_integral_sum(&gps_longitude_pid);
        pid_reset_integral_sum(&gps_latitude_pid);

        error_latitude = 0.0;
        error_longitude = 0.0;

        gps_hold_roll_adjustment = 0.0;
        gps_hold_pitch_adjustment = 0.0;

        if(got_gps){
            target_latitude = gps_latitude;
            target_longitude = gps_longitude;
            gps_target_unset_logged = 0;
            gps_target_unset_cause = 2;
        }

        // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

        //Set new target for the altitude
        target_altitude_barometer = altitude_barometer;
        // gyro_yaw = magnetometer_yaw;
        // gyro_yaw_old = magnetometer_yaw_old;

        // Immediately after getting the motor values send them to motors. If manual mode is on
        if(manual_bdshot) bdshot600_dma_send_all_motor_data();
        motor7 = DWT->CYCCNT;
        return;
    }
    // When receiving signals, decrease the complementary filter ratio to make the drone more responsive to gyro
    mpu6050_set_complementary_ratio(COMPLEMENTARY_RATIO_MULTIPLYER_FLIGHT * (1.0 - 1.0/(1.0+(1.0/REFRESH_RATE_HZ))));

    // ---------------------------------------------------------------------------------------------- GPS position hold
    if(use_gps_hold){
        // If the gps is not up to date then do not use it
        if(gps_position_hold_enabled && gps_can_be_used){
            pid_set_desired_value(&gps_latitude_pid, 0.0);
            pid_set_desired_value(&gps_longitude_pid, 0.0);

            float roll_command = pid_get_error_own_error(&gps_longitude_pid, error_right, loop_start_timestamp_microseconds);
            float pitch_command = pid_get_error_own_error(&gps_latitude_pid, error_forward, loop_start_timestamp_microseconds);

            // For logging
            PID_proportional[4] = pid_get_last_proportional_error(&gps_latitude_pid);
            PID_integral[4] = pid_get_last_integral_error(&gps_latitude_pid);
            PID_derivative[4] = pid_get_last_derivative_error(&gps_latitude_pid);

            // Convert the roll and pitch adjustment to a vector
            // This makes the target position more clear to the drone
            // Roll and pitch is adjusted to go STRAIGHT to the target
            float gps_hold_vector_magnitude = sqrtf(roll_command * roll_command + pitch_command * pitch_command);

            // Get unit vectors of error lat and lon
            float unit_vector_roll_command = 0.0f;
            float unit_vector_pitch_command = 0.0f;
            if(gps_hold_vector_magnitude != 0.0f){
                unit_vector_roll_command = roll_command / gps_hold_vector_magnitude;
                unit_vector_pitch_command = pitch_command / gps_hold_vector_magnitude;
            }

            float scale_factor = fmin(gps_hold_vector_magnitude, gps_pid_angle_of_attack_max);

            unit_vector_roll_command = unit_vector_roll_command * scale_factor;
            unit_vector_pitch_command = unit_vector_pitch_command * scale_factor;

            // Works better inverted
            gps_hold_roll_adjustment = -unit_vector_roll_command;
            gps_hold_pitch_adjustment = -unit_vector_pitch_command;

        }else if(gps_position_hold_enabled && !gps_can_be_used){
            // Put in the same values'
            gps_hold_roll_adjustment = 0.0f;
            gps_hold_pitch_adjustment = 0.0f;

            pid_reset_integral_sum(&gps_latitude_pid);
            pid_reset_integral_sum(&gps_longitude_pid);

            // Reason for gps off
            if(gps_target_unset_logged != 0){
                gps_target_unset_logged = 0;
                gps_target_unset_cause = 3;
            }
        }else{
            gps_hold_roll_adjustment = 0.0f;
            gps_hold_pitch_adjustment = 0.0f;

            pid_reset_integral_sum(&gps_latitude_pid);
            pid_reset_integral_sum(&gps_longitude_pid);

            if(gps_target_unset_logged != 0){
                gps_target_unset_logged = 0;
                gps_target_unset_cause = 4;
            }
        }
    }
    motor2 = DWT->CYCCNT;
    

    // ---------------------------------------------------------------------------------------------- Altitude hold
    if(use_vertical_velocity_control && got_pressure){
        pid_set_desired_value(&altitude_hold_pid, target_altitude_barometer);
        pid_calculate_error(&altitude_hold_pid, altitude_barometer, loop_start_timestamp_microseconds);

        PID_proportional[3] = pid_get_last_proportional_error(&altitude_hold_pid);
        PID_integral[3] = pid_get_last_integral_error(&altitude_hold_pid);
        PID_derivative[3] = pid_get_last_derivative_error(&altitude_hold_pid);

        PID_set_points[4] = target_altitude_barometer;
        error_altitude_barometer = PID_proportional[3] + PID_integral[3] + PID_derivative[3];
    }

    motor3 = DWT->CYCCNT;

    // ---------------------------------------------------------------------------------------------- Angle mode
    if(use_angle_mode){ 
        pid_set_desired_value(&angle_roll_pid, angle_target_roll + gps_hold_roll_adjustment);
        pid_set_desired_value(&angle_pitch_pid, angle_target_pitch + gps_hold_pitch_adjustment);

        // For now not logging desired angle mode
        error_angle_roll = limit_max_value(pid_get_error(&angle_roll_pid, imu_orientation[0], loop_start_timestamp_microseconds), -acro_mode_rate_degrees_per_second, acro_mode_rate_degrees_per_second);
        error_angle_pitch = limit_max_value(pid_get_error(&angle_pitch_pid, imu_orientation[1], loop_start_timestamp_microseconds), -acro_mode_rate_degrees_per_second, acro_mode_rate_degrees_per_second);

    }else{
        error_angle_roll = 0;
        error_angle_pitch = 0;
    }

    motor4 = DWT->CYCCNT;

    // ---------------------------------------------------------------------------------------------- Acro mode
    if(use_angle_mode){
        pid_set_desired_value(&acro_roll_pid, error_angle_roll);
        pid_set_desired_value(&acro_pitch_pid, error_angle_pitch);
        pid_set_desired_value(&acro_yaw_pid, acro_target_yaw);
        
        PID_set_points[0] = error_angle_roll; // Logging
        PID_set_points[1] = error_angle_pitch; // Logging
        PID_set_points[2] = acro_target_yaw; // Logging
    }else{
        pid_set_desired_value(&acro_roll_pid, acro_target_roll);
        pid_set_desired_value(&acro_pitch_pid, acro_target_pitch);
        pid_set_desired_value(&acro_yaw_pid, acro_target_yaw);
        
        PID_set_points[0] = acro_target_roll; // Logging
        PID_set_points[1] = acro_target_pitch; // Logging
        PID_set_points[2] = acro_target_yaw; // Logging
    }
    

    // Just calculate and not get the error sum. Needed for d term filter
    pid_calculate_error_pi(&acro_roll_pid, gyro_angular[0], loop_start_timestamp_microseconds); // Calculate P and I terms
    pid_calculate_error_d(&acro_roll_pid, gyro_angular_for_d_term[0], loop_start_timestamp_microseconds); // Calculate D term using a low pass filtered value
    pid_set_previous_time(&acro_roll_pid, loop_start_timestamp_microseconds); // Update last timestamp

    pid_calculate_error_pi(&acro_pitch_pid, gyro_angular[1], loop_start_timestamp_microseconds);
    pid_calculate_error_d(&acro_pitch_pid, gyro_angular_for_d_term[1], loop_start_timestamp_microseconds);
    pid_set_previous_time(&acro_pitch_pid, loop_start_timestamp_microseconds);

    pid_calculate_error(&acro_yaw_pid, gyro_angular[2], loop_start_timestamp_microseconds);
    pid_set_previous_time(&acro_yaw_pid, loop_start_timestamp_microseconds);

    // printf("ROl g: %.3f gf: %.3f dt: %.3f \n", gyro_angular[0], gyro_angular_for_d_term[0], PID_derivative[0]);

    // Inner pid roll logging
    PID_proportional[0] = pid_get_last_proportional_error(&acro_roll_pid);
    PID_integral[0] = pid_get_last_integral_error(&acro_roll_pid);
    PID_derivative[0] = pid_get_last_derivative_error(&acro_roll_pid);

    // Inner pid pitch logging
    PID_proportional[1] = pid_get_last_proportional_error(&acro_pitch_pid);
    PID_integral[1] = pid_get_last_integral_error(&acro_pitch_pid);
    PID_derivative[1] = pid_get_last_derivative_error(&acro_pitch_pid);

    // Inner pid yaw logging
    PID_proportional[2] = pid_get_last_proportional_error(&acro_yaw_pid);
    PID_integral[2] = pid_get_last_integral_error(&acro_yaw_pid);




    // Use throttle to make the biquad filter dynamic
    error_acro_pitch = PID_proportional[1] + PID_integral[1] + PID_derivative[1];
    error_acro_roll = PID_proportional[0] + PID_integral[0] + PID_derivative[0];
    error_acro_yaw = PID_proportional[2] + PID_integral[2];

    if(blackbox_log_angle_mode_pid){
        // Outer pid roll logging
        PID_proportional[0] = pid_get_last_proportional_error(&angle_roll_pid);
        PID_integral[0] = pid_get_last_integral_error(&angle_roll_pid);
        PID_derivative[0] = pid_get_last_derivative_error(&angle_roll_pid);

        // Outer pid pitch logging
        PID_proportional[1] = pid_get_last_proportional_error(&angle_pitch_pid);
        PID_integral[1] = pid_get_last_integral_error(&angle_pitch_pid);
        PID_derivative[1] = pid_get_last_derivative_error(&angle_pitch_pid);
    }


    motor5 = DWT->CYCCNT;

    // ---------------------------------------------------------------------------------------------- Throttle
    error_throttle = throttle*0.9;

    // ---------------------------------------------------------------------------------------------- Applying to throttle to escs
    if(!use_disable_all_control){
        motor_power[0] = ( error_acro_roll) + ( error_acro_pitch) + ( error_acro_yaw);
        motor_power[1] = (-error_acro_roll) + ( error_acro_pitch) + (-error_acro_yaw);
        motor_power[2] = (-error_acro_roll) + (-error_acro_pitch) + ( error_acro_yaw);
        motor_power[3] = ( error_acro_roll) + (-error_acro_pitch) + (-error_acro_yaw);
    }else{
        motor_power[0] = 0.0;
        motor_power[1] = 0.0;
        motor_power[2] = 0.0;
        motor_power[3] = 0.0;
    }

    // For throttle control logic

    throttle_value = error_throttle * (~use_vertical_velocity_control & 1) + error_altitude_barometer * use_vertical_velocity_control;
    motor_power[0] += throttle_value;
    motor_power[1] += throttle_value;
    motor_power[2] += throttle_value;
    motor_power[3] += throttle_value;

    // For logging throttle value
    PID_set_points[3] = setServoActivationPercent(throttle_value, actual_min_dshot600_throttle_value, actual_max_dshot600_throttle_value);

    throttle_value_FR = setServoActivationPercent(motor_power[2], actual_min_dshot600_throttle_value, actual_max_dshot600_throttle_value);
    throttle_value_FL = setServoActivationPercent(motor_power[3], actual_min_dshot600_throttle_value, actual_max_dshot600_throttle_value);
    throttle_value_BL = setServoActivationPercent(motor_power[0], actual_min_dshot600_throttle_value, actual_max_dshot600_throttle_value);
    throttle_value_BR = setServoActivationPercent(motor_power[1], actual_min_dshot600_throttle_value, actual_max_dshot600_throttle_value);

    // FPV cam side FRONT
    bdshot600_dma_set_throttle(throttle_value_FR, motor_FR); // Front-right
    bdshot600_dma_set_throttle(throttle_value_FL, motor_FL); // Front-left
    // GPS side BACK
    bdshot600_dma_set_throttle(throttle_value_BL, motor_BL); // Back-left
    bdshot600_dma_set_throttle(throttle_value_BR, motor_BR); // Back-right

    // For logging
    motor_power[0] = throttle_value_BL; // Logging BL <- BL
    motor_power[1] = throttle_value_FL; // Logging FL <- FL
    motor_power[2] = throttle_value_BR; // Logging BR <- BR
    motor_power[3] = throttle_value_FR; // Logging FR <- FR

    motor6 = DWT->CYCCNT;

    // If manual mode is on. Immediately after getting the motor values, send them to motors.
    if(manual_bdshot) bdshot600_dma_send_all_motor_data();
    motor7 = DWT->CYCCNT;
}


// 0.5ms to 2ms = range is 1.5ms
// 0.5 is 2.5%  and is 25 in the float value passed to this
// 2   is 10%   amd is 100 in the float value passed to this
// 100 - 25 = 75

// default min and max is 25 and 75
uint16_t setServoActivationPercent(float percent, uint16_t minValue, uint16_t maxValue){
    // Kind of annoying having these if statements
    if(percent > 100.0){
        percent = 100.0f;
    }else if(percent < 0.0){
        percent = 0.0;
    }
    return (percent / 100.0f) * (maxValue - minValue) + minValue;
}

float limit_max_value(float value, float min_value, float max_value){

    if(value > max_value){
        return max_value;
    }else if(value < min_value){
        return min_value;
    }else{
        return value;
    }
}
