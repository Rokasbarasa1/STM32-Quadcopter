#include "./startup.h"

void startup_procedure(){
    accelerometer_roll_offset = base_accelerometer_roll_offset;
    accelerometer_pitch_offset = base_accelerometer_pitch_offset;
    // Turn off the blue led. Will show the status of sd logging later
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    printf("STARTING PROGRAM\n"); 
    
    HAL_Delay(500);
    
    if(init_drivers() == 0) return 0; // exit if initialization failed

    HAL_Delay(2000); // Dont want to interfere in calibration

    set_flight_mode(flight_mode);
    // check_calibrations();
    calibrate_gyro(); // Recalibrate the gyro as the temperature affects the calibration
    setup_logging_to_sd(0); // If stopped working then re-flash the firmware of the logger and check hardware connection. Check if it works when Li-po connected
    initialize_control_abstractions();
    initialize_motor_communication();
    get_initial_position();
}

uint8_t init_drivers(){
    printf("-----------------------------INITIALIZING MODULES...\n");
    // Scan and see what i2c devices are there on the bus
    // i2c_scan_bus(&hi2c1);

    mmc5603 = mmc5603_init(
        &hi2c1, 
        1, 
        magnetometer_mmc5603_hard_iron_correction, 
        magnetometer_mmc5603_soft_iron_correction, 
        1, 
        200, 
        1
    );
    
    // qmc5883 = init_qmc5883l(
    //     &hi2c1, 
    //     1, 
    //     magnetometer_hard_iron_correction, 
    //     magnetometer_soft_iron_correction
    // );
    // hmc5883 = init_hmc5883l(
    //     &hi2c1, 
    //     1, 
    //     magnetometer_hard_iron_correction, 
    //     magnetometer_soft_iron_correction
    // );
    bmm350 = bmm350_init(
        &hi2c1, 
        1, 
        magnetometer_hard_iron_correction, 
        magnetometer_soft_iron_correction,
        BMM350_PAD_CTRL_ODR_400_HZ,
        BMM350_PAD_CTRL_AVG_NO_AVG
    );
    ist8310 = ist8310_init(
        &hi2c1, 
        1, 
        magnetometer_ist8310_hard_iron_correction, 
        magnetometer_ist8310_soft_iron_correction
    );

    mpu6050 = init_mpu6050(
        &hi2c1, 
        ACCEL_CONFIG_RANGE_2G, 
        GYRO_CONFIG_RANGE_500_DEG, 
        LOW_PASS_FILTER_FREQUENCY_CUTOFF_GYRO_260HZ_ACCEL_256HZ,
        1, 
        accelerometer_scale_factor_correction, 
        accelerometer_correction, 
        gyro_correction, 
        REFRESH_RATE_HZ, 
        complementary_ratio
    );


    ms5611 = init_ms5611(&hi2c1);

    bn357 = init_bn357(&huart2, &hdma_usart2_rx, 1);
    nrf24 = init_nrf24(&hspi1, 1);

    printf("-----------------------------INITIALIZING MODULES DONE... ");
    if (mpu6050 && ms5611 && bn357 && nrf24){
        printf("OK\n");
    }else{
        printf("NOT OK\n");
        return 0;
    }

    // Initialize ms5611
    if(ms5611){
        ms5611_min_pause_after_conversion_initiate_microseconds = ms5611_conversion_wait_time_microseconds();
        ms5611_set_prefered_data_conversion_preasure(MS5611_D1_CONVERSION_OSR_4096);
        ms5611_set_prefered_data_conversion_temperature(MS5611_D2_CONVERSION_OSR_4096);
    
        // Get the temperature
        ms5611_initiate_prefered_temperature_conversion();
        HAL_Delay(10); // 10ms delay to let the conversion finish
        temperature_celsius = ms5611_read_conversion_temperature_celsius();

        // Get the preasure
        ms5611_set_reference_pressure_from_number_of_samples(40);
        pressure_hpa = ms5611_read_conversion_preasure_hPa();

        // Start new preasure reading
        ms5611_initiate_prefered_preasure_conversion();
        ms5611_conversion_start_timestamp = get_absolute_time();
    }

    if(nrf24){
        nrf24_rx_mode(tx_address, 10);
    }

    if(bn357){
        bn357_start_uart_interrupt();
    }

    if(mpu6050){
        mpu6050_set_complementary_ratio(COMPLEMENTARY_RATIO_MULTIPLYER_OFF * (1.0 - 1.0/(1.0+(1.0/REFRESH_RATE_HZ))));
        
        // mpu6050_set_complementary_ratio_yaw(COMPLEMENTARY_RATIO_MULTIPLYER_YAW_UPPER_GATE_MORE_MAG * (1.0 - 1.0/(1.0+(1.0/REFRESH_RATE_HZ))));
        // yaw_alpha = COMPLEMENTARY_RATIO_MULTIPLYER_YAW_UPPER_GATE_MORE_MAG * (1.0 - 1.0/(1.0+(1.0/REFRESH_RATE_HZ)));

        mpu6050_set_complementary_ratio_yaw(COMPLEMENTARY_RATIO_MULTIPLYER_YAW_UPPER_GATE_MORE_MAG);
        yaw_alpha = COMPLEMENTARY_RATIO_MULTIPLYER_YAW_UPPER_GATE_MORE_MAG;
    }

    return 1;
}






void initialize_control_abstractions(){
    // PID controllers
    acro_roll_pid = pid_init(acro_roll_pitch_master_gain * acro_roll_pitch_gain_p, acro_roll_pitch_master_gain * acro_roll_pitch_gain_i, acro_roll_pitch_master_gain * acro_roll_pitch_gain_d, 0.0, get_absolute_time(), 25, -25, 1, 0);
    acro_pitch_pid = pid_init(acro_roll_pitch_master_gain * acro_roll_pitch_gain_p, acro_roll_pitch_master_gain * acro_roll_pitch_gain_i, acro_roll_pitch_master_gain * acro_roll_pitch_gain_d, 0.0, get_absolute_time(), 25, -25, 1, 0);
    acro_yaw_pid = pid_init(acro_yaw_gain_p * acro_yaw_gain_master, acro_yaw_gain_i * acro_yaw_gain_master, 0, 0.0, get_absolute_time(), 10.0, -10.0, 1, 0);

    angle_roll_pid = pid_init(angle_roll_pitch_master_gain * angle_roll_pitch_gain_p, angle_roll_pitch_master_gain * angle_roll_pitch_gain_i, angle_roll_pitch_master_gain * angle_roll_pitch_gain_d, 0.0, get_absolute_time(), angle_mode_rate_degrees_per_second_max_integral_derivative, -angle_mode_rate_degrees_per_second_max_integral_derivative, 1, 0);
    angle_pitch_pid = pid_init(angle_roll_pitch_master_gain * angle_roll_pitch_gain_p, angle_roll_pitch_master_gain * angle_roll_pitch_gain_i, angle_roll_pitch_master_gain * angle_roll_pitch_gain_d, 0.0, get_absolute_time(), angle_mode_rate_degrees_per_second_max_integral_derivative, -angle_mode_rate_degrees_per_second_max_integral_derivative, 1, 0);

    altitude_hold_pid = pid_init(altitude_hold_master_gain * altitude_hold_gain_p, altitude_hold_master_gain * altitude_hold_gain_i, altitude_hold_master_gain * altitude_hold_gain_d, 0.0, get_absolute_time(), 10.0, -0.0, 1, 0); // Min value is 0 because motors dont go lower that that

    gps_longitude_pid = pid_init(gps_hold_master_gain * gps_hold_gain_p, gps_hold_master_gain * gps_hold_gain_i, gps_hold_master_gain * gps_hold_gain_d, 0.0, get_absolute_time(), 2.0, -2.0, 1, 0);
    gps_latitude_pid = pid_init(gps_hold_master_gain * gps_hold_gain_p, gps_hold_master_gain * gps_hold_gain_i, gps_hold_master_gain * gps_hold_gain_d, 0.0, get_absolute_time(), 2.0, -2.0, 1, 0);

    // Filtering
    filter_magnetometer_x = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);// TODO bmm350 has refresh rate of 400
    filter_magnetometer_y = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);
    filter_magnetometer_z = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);

    filter_magnetometer_x_secondary = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);
    filter_magnetometer_y_secondary = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);
    filter_magnetometer_z_secondary = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);

    filter_magnetometer_90_x = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);
    filter_magnetometer_90_y = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);
    filter_magnetometer_90_z = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);

    filter_magnetometer_180_x = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);
    filter_magnetometer_180_x = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);
    filter_magnetometer_180_x = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);

    filter_magnetometer_270_x = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);
    filter_magnetometer_270_x = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);
    filter_magnetometer_270_x = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);

    
    filter_magnetometer_ist8310_x = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, 200);
    filter_magnetometer_ist8310_y = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, 200);
    filter_magnetometer_ist8310_z = filtering_init_low_pass_filter_biquad(filtering_magnetometer_cutoff_frequency, 200);

    // Magnetometer filtering
    iir_filter_magnetometer_x = iir_filter_init(magnetometer_iir_filter_order, magnetometer_iir_filter_feedback, magnetometer_iir_filter_feedforward);
    iir_filter_magnetometer_y = iir_filter_init(magnetometer_iir_filter_order, magnetometer_iir_filter_feedback, magnetometer_iir_filter_feedforward);
    iir_filter_magnetometer_z = iir_filter_init(magnetometer_iir_filter_order, magnetometer_iir_filter_feedback, magnetometer_iir_filter_feedforward);

    moving_average_magnetometer_x = moving_average_init(moving_average_magnetometer_size);
    moving_average_magnetometer_y = moving_average_init(moving_average_magnetometer_size);
    moving_average_magnetometer_z = moving_average_init(moving_average_magnetometer_size);

    outlier_detection_magnetometer_x = outlier_detection_init(outlier_detection_magnetometer_window_size, outlier_detection_magnetometer_threshold);
    outlier_detection_magnetometer_y = outlier_detection_init(outlier_detection_magnetometer_window_size, outlier_detection_magnetometer_threshold);
    outlier_detection_magnetometer_z = outlier_detection_init(outlier_detection_magnetometer_window_size, outlier_detection_magnetometer_threshold);


    biquad_filter_accelerometer_x = filtering_init_low_pass_filter_biquad(filtering_accelerometer_cutoff_frequency, REFRESH_RATE_HZ);
    biquad_filter_accelerometer_y = filtering_init_low_pass_filter_biquad(filtering_accelerometer_cutoff_frequency, REFRESH_RATE_HZ);
    biquad_filter_accelerometer_z = filtering_init_low_pass_filter_biquad(filtering_accelerometer_cutoff_frequency, REFRESH_RATE_HZ);
    // Yaw filtering 
    filter_gyro_z = filtering_init_low_pass_filter_pt1(filter_gyro_z_yaw_cutoff_frequency, REFRESH_RATE_HZ);


    filter_motor_0 = filtering_init_low_pass_filter_pt1(motor_low_pass_filter_cutoff, REFRESH_RATE_HZ);
    filter_motor_1 = filtering_init_low_pass_filter_pt1(motor_low_pass_filter_cutoff, REFRESH_RATE_HZ);
    filter_motor_2 = filtering_init_low_pass_filter_pt1(motor_low_pass_filter_cutoff, REFRESH_RATE_HZ);
    filter_motor_3 = filtering_init_low_pass_filter_pt1(motor_low_pass_filter_cutoff, REFRESH_RATE_HZ);

    // RPM filter
    // Width does not matter and frequency will be set later using data from motors
    gyro_x_notch_filter_harmonic_1_motor_0 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_1, REFRESH_RATE_HZ);
    gyro_x_notch_filter_harmonic_1_motor_1 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_1, REFRESH_RATE_HZ);
    gyro_x_notch_filter_harmonic_1_motor_2 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_1, REFRESH_RATE_HZ);
    gyro_x_notch_filter_harmonic_1_motor_3 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_1, REFRESH_RATE_HZ);

    gyro_y_notch_filter_harmonic_1_motor_0 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_1, REFRESH_RATE_HZ);
    gyro_y_notch_filter_harmonic_1_motor_1 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_1, REFRESH_RATE_HZ);
    gyro_y_notch_filter_harmonic_1_motor_2 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_1, REFRESH_RATE_HZ);
    gyro_y_notch_filter_harmonic_1_motor_3 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_1, REFRESH_RATE_HZ);

    gyro_x_notch_filter_harmonic_3_motor_0 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_3, REFRESH_RATE_HZ);
    gyro_x_notch_filter_harmonic_3_motor_1 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_3, REFRESH_RATE_HZ);
    gyro_x_notch_filter_harmonic_3_motor_2 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_3, REFRESH_RATE_HZ);
    gyro_x_notch_filter_harmonic_3_motor_3 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_3, REFRESH_RATE_HZ);

    gyro_y_notch_filter_harmonic_3_motor_0 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_3, REFRESH_RATE_HZ);
    gyro_y_notch_filter_harmonic_3_motor_1 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_3, REFRESH_RATE_HZ);
    gyro_y_notch_filter_harmonic_3_motor_2 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_3, REFRESH_RATE_HZ);
    gyro_y_notch_filter_harmonic_3_motor_3 = notch_filter_q_init(0, rpm_notch_filter_q_harmonic_3, REFRESH_RATE_HZ);

    // D-term biquad low-pass filter
    // This is an AOS D term tune
    // Initialize with the minimum frequency
    roll_d_term_filtering = filtering_init_low_pass_filter_biquad(d_term_filtering_min_cutoff, REFRESH_RATE_HZ);
    pitch_d_term_filtering = filtering_init_low_pass_filter_biquad(d_term_filtering_min_cutoff, REFRESH_RATE_HZ);


    altitude_barometer_filtering = filtering_init_low_pass_filter_biquad(
        altitude_barometer_filtering_min_cutoff, 
        (uint16_t)( // This calculates the refresh rate in Hz. One loop time x 2 + 2 times the time it takes to read the sensor
            1000000.0f / (
                (1000000.0f / (float)REFRESH_RATE_HZ - 1.0f) * 2.0f +
                2.0f * (float)ms5611_min_pause_after_conversion_initiate_microseconds
            )
        )
    );


    // biquad_filter_gps_lat = filtering_init_low_pass_filter_biquad(gps_filtering_min_cutoff, 10);
    // biquad_filter_gps_lon = filtering_init_low_pass_filter_biquad(gps_filtering_min_cutoff, 10);

    
    biquad_filter_gps_lat = filtering_init_low_pass_filter_pt1(gps_filtering_min_cutoff, 10);
    biquad_filter_gps_lon = filtering_init_low_pass_filter_pt1(gps_filtering_min_cutoff, 10);


    // Kalman  sensor fusion for vertical velocity
    float S_state[2][1] = {
        {0},
        {0}
    };

    float F_state_transition[2][2] = {
        {1, 0.005},
        {0, 1}
    };

    float P_covariance[2][2] = {
        {0, 0},
        {0, 0}
    }; 

    float B_control[2][1] = {
        {0.5 + (0.005 * 0.005)},
        {0.005}
    };

    float H_observation[1][2] = {
        {1, 0}
    };

    float Q_proccess_uncertainty[2][2] = {
        {25.0025001, 0.2500125},
        {0.2500125, 0.0025}
    };

    float R_measurement_uncertainty[1][1] = {
        {30 * 30}
    };

    altitude_and_velocity_kalman = kalman_filter_init();
    kalman_filter_set_S_state(&altitude_and_velocity_kalman, (float *)S_state, 2);
    kalman_filter_set_P_covariance(&altitude_and_velocity_kalman, (float *)P_covariance, 2, 2);
    kalman_filter_set_F_state_transition_model(&altitude_and_velocity_kalman, (float *)F_state_transition, 2, 2);
    kalman_filter_set_Q_proccess_noise_covariance(&altitude_and_velocity_kalman, (float *)Q_proccess_uncertainty, 2, 2);
    kalman_filter_set_H_observation_model(&altitude_and_velocity_kalman, (float *)H_observation, 1, 2);
    kalman_filter_set_B_external_input_control_matrix(&altitude_and_velocity_kalman, (float *)B_control, 2, 1);
    kalman_filter_set_R_observation_noise_covariance(&altitude_and_velocity_kalman, (float *)R_measurement_uncertainty, 1, 1);
    kalman_filter_configure_u_measurement_input(&altitude_and_velocity_kalman, 1);
    kalman_filter_configure_z_measurement_input(&altitude_and_velocity_kalman, 1);
    uint8_t altitude_kalman_status = kalman_filter_finish_initialization(&altitude_and_velocity_kalman);

    if(altitude_kalman_status == 1) printf("Kalman setup OK\n");
    else printf("Kalman setup not OK\n");
}

void check_calibrations(){
    HAL_Delay(2000);
    // Checking errors of mpu6050
    find_accelerometer_error(1000);
    find_gyro_error(1000);
}

void set_flight_mode(uint8_t mode){
    if(mode == 0){
        use_disable_all_control = 1;
        use_angle_mode = 0;
        use_vertical_velocity_control = 0;
        use_gps_hold = 0;
    }else if(mode == 1){
        use_disable_all_control = 0;
        use_angle_mode = 0;
        use_vertical_velocity_control = 0;
        use_gps_hold = 0;
    }else if(mode == 2){
        use_disable_all_control = 0;
        use_angle_mode = 1;
        use_vertical_velocity_control = 0;
        use_gps_hold = 0;
    }else if(mode == 3){
        use_disable_all_control = 0;
        use_angle_mode = 1;
        use_vertical_velocity_control = 1;
        use_gps_hold = 0;
    }else if(mode == 4){
        use_disable_all_control = 0;
        use_angle_mode = 1;
        use_vertical_velocity_control = 1;
        use_gps_hold = 1;
    }else if(mode == 5){
        use_disable_all_control = 0;
        use_angle_mode = 1;
        use_vertical_velocity_control = 0;
        use_gps_hold = 1;
    }
}


void get_initial_position(){
    // Find the initial position in degrees and apply it to the gyro measurement integral
    // This will tell the robot which way to go to get the actual upward
    __disable_irq();
    bmm350_magnetometer_readings_micro_teslas(magnetometer_data, 1);
    mmc5603_magnetometer_readings_micro_teslas(magnetometer_data_secondary);

    // WILL FIX THIS LATER


    // qmc5883l_magnetometer_readings_micro_teslas(magnetometer_data);
    // hmc5883l_magnetometer_readings_micro_teslas(magnetometer_data);


    mpu6050_get_accelerometer_readings_gravity(acceleration_data);
    mpu6050_get_gyro_readings_dps(gyro_angular);
    __enable_irq();

    invert_axies(acceleration_data);
    invert_axies(gyro_angular);
    // rotate_magnetometer_vector(magnetometer_data, -16.8f, 16.8f, -90.0f);
    // rotate_magnetometer_output_90_degrees_anti_clockwise(magnetometer_data);

    // magnetometer_data_current_unfiltered[0] = magnetometer_data[0];
    // magnetometer_data_current_unfiltered[1] = magnetometer_data[1];
    // magnetometer_data_current_unfiltered[2] = magnetometer_data[2];

    // magnetometer_low_pass[0] = low_pass_filter_biquad_read(&filter_magnetometer_x, magnetometer_data[0]);
    // magnetometer_low_pass[1] = low_pass_filter_biquad_read(&filter_magnetometer_y, magnetometer_data[1]);
    // magnetometer_low_pass[2] = low_pass_filter_biquad_read(&filter_magnetometer_z, magnetometer_data[2]);

    // magnetometer_data[0] = outlier_detection_process(&outlier_detection_magnetometer_x, magnetometer_data[0]);
    // magnetometer_data[1] = outlier_detection_process(&outlier_detection_magnetometer_y, magnetometer_data[1]);
    // magnetometer_data[2] = outlier_detection_process(&outlier_detection_magnetometer_z, magnetometer_data[2]);

    // magnetometer_data[0] = iir_filter_process(&iir_filter_magnetometer_x, magnetometer_data[0]);
    // magnetometer_data[1] = iir_filter_process(&iir_filter_magnetometer_y, magnetometer_data[1]);
    // magnetometer_data[2] = iir_filter_process(&iir_filter_magnetometer_z, magnetometer_data[2]);

    // magnetometer_data[0] = moving_average_process(&moving_average_magnetometer_x, magnetometer_data[0]);
    // magnetometer_data[1] = moving_average_process(&moving_average_magnetometer_y, magnetometer_data[1]);
    // magnetometer_data[2] = moving_average_process(&moving_average_magnetometer_z, magnetometer_data[2]);

    calculate_roll_pitch_from_accelerometer_data(acceleration_data, &accelerometer_roll, &accelerometer_pitch, accelerometer_roll_offset, accelerometer_pitch_offset);
    // calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data_current_unfiltered, &mangetometer_yaw_unfiltered, imu_orientation[0], imu_orientation[1], yaw_offset + yaw_declination);
    // calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_low_pass, &mangetometer_yaw_low_pass, imu_orientation[0], imu_orientation[1], yaw_offset + yaw_declination);
    // calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data, &magnetometer_yaw, accelerometer_roll, accelerometer_pitch, yaw_offset + yaw_declination);

    
    imu_orientation[0] = accelerometer_roll;
    imu_orientation[1] = accelerometer_pitch;
    // imu_orientation[2] = mangetometer_yaw_low_pass; // For some reason the more advanced filtering returns nan here so low pass is fine
    printf("Initial imu orientation x: %.2f y: %.2f, z: %.2f\n", imu_orientation[0], imu_orientation[1], imu_orientation[2]);
}

void calibrate_gyro(){
    // Gyro can be calibrated when it is standing still. Not necessarily on flat ground.
    float gyro_correction_temp[3] = {
        0, 0, 0
    };  

    find_and_return_gyro_error(1000, gyro_correction_temp);
    mpu6050_apply_calibrations_gyro(gyro_correction_temp);
}



float triangle_sin(float x){
    return triangle_wave(x);
}

float triangle_cos(float x){
    return triangle_wave(x + M_HALF_PI);
}




void switch_x_and_y_axis(float *data){
    float temp = 0.0;
    temp = data[0];
    // y is x
    data[0] = data[1];
    // x is -y
    data[1] = temp;
}



float sawtooth_sin(float x_radian){
    return 2.0f * (x_radian*M_DIVIDE_BY_PI - floorf(x_radian*M_DIVIDE_BY_PI + 0.5f));
}

float sawtooth_cos(float x_radian){
    return 2.0f * ((x_radian + M_HALF_PI)*M_DIVIDE_BY_PI - floorf(x_radian*M_DIVIDE_BY_PI + 0.5f));
}

float triangle_wave(float x) {
    // Normalize x to be within the range [0, 2*PI)
    x = fmodf(x, M_PI_TWO);
    
    // Scale the normalized x to the range [0, 4]
    float scaled_x = x  * M_DIVIDE_BY_HALF_PI;
    
    // Triangle wave calculation
    if (scaled_x < 1.0f) return scaled_x;
    else if (scaled_x < 3.0f) return 2.0f - scaled_x;
    else return scaled_x - 4.0f;
}
