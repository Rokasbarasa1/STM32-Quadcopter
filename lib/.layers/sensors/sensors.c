#include "./sensors.h"

void handle_get_and_calculate_sensor_values(){
    // ------------------------------------------------------------------------------------------------------ Reset
    // Reset the values to make it easier to find broken sensor later
    reset_array_data(acceleration_data, 3);
    reset_array_data(gyro_angular, 3);
    reset_array_data(magnetometer_data, 3);
    got_pressure = 0;
    // temperature_celsius = 0.0f;
    // altitude_barometer = 0.0f;
    // vertical_velocity = 0.0f;

    // ------------------------------------------------------------------------------------------------------ Get the sensor data
    // Get data from motors
    if(bdshot600_dma_convert_all_responses(1)){
        // Motor 0 - BL, 1 - BR, 2 - FR, 3 - FL
        motor_frequency[0] = low_pass_filter_pt1_read(&filter_motor_0, bdshot600_dma_get_frequency(motor_BL));
        motor_frequency[1] = low_pass_filter_pt1_read(&filter_motor_1, bdshot600_dma_get_frequency(motor_BR));
        motor_frequency[2] = low_pass_filter_pt1_read(&filter_motor_2, bdshot600_dma_get_frequency(motor_FR));
        motor_frequency[3] = low_pass_filter_pt1_read(&filter_motor_3, bdshot600_dma_get_frequency(motor_FL));
    }

    sensors2 = DWT->CYCCNT;

    // Disable interrupts as they heavily impact the i2c communication
    __disable_irq();
    // Read preasure strictly every around 10 ms
    if(get_absolute_time() - ms5611_conversion_start_timestamp > ms5611_min_pause_after_conversion_initiate_microseconds){
        if(ms5611_which_conversion == 0){
            temperature_celsius = ms5611_read_conversion_temperature_celsius();
            ms5611_initiate_prefered_preasure_conversion();
            ms5611_which_conversion = 1;
        }else if(ms5611_which_conversion == 1){
            pressure_hpa = ms5611_read_conversion_preasure_hPa();
            got_pressure = 1;

            // ONLY ONCE, set the reference after some time into the loop
            if(ms5611_reference_set == 0 && get_absolute_time() - ms5611_loop_start_timestamp > ms5611_set_reference_pressure_after_microseconds_of_loop){
                ms5611_reference_set = 1;
                altitude_barometer = ms5611_get_height_centimeters_from_reference(1, 0, 1);
            }else{
                // NORMAL call
                altitude_barometer = ms5611_get_height_centimeters_from_reference(1, 0, 0);
            }
            ms5611_initiate_prefered_temperature_conversion();
            ms5611_which_conversion = 0;
        }
        ms5611_conversion_start_timestamp = get_absolute_time();
    }
    
    // if(ist8310 && get_absolute_time() - last_magnetometer_reading_timestamp_microseconds > magnetometer_reading_timestamp_time_microseconds){
    //     last_magnetometer_reading_timestamp_microseconds = get_absolute_time();
    //     ist8310_magnetometer_readings_micro_teslas(magnetometer_data_ist8310, 1);
    //     ist8310_magnetometer_initiate_reading();
    //     got_ist8310_reading = 1;
    // }else{
    //     got_ist8310_reading = 0;
    // }
    // qmc5883l_magnetometer_readings_micro_teslas(magnetometer_data);
    // hmc5883l_magnetometer_readings_micro_teslas(magnetometer_data);

    mmc5603_magnetometer_readings_micro_teslas(magnetometer_data_secondary, 1);
    bmm350_magnetometer_readings_micro_teslas(magnetometer_data, 1, 1);

    mpu6050_get_accelerometer_readings_gravity(acceleration_data);
    mpu6050_get_gyro_readings_dps(gyro_angular);
    __enable_irq();

    bmm350_magnetometer_readings_micro_teslas_unrotated(magnetometer_data_unrotated);
    mmc5603_magnetometer_readings_micro_teslas_unrotated(magnetometer_data_secondary_unrotated);

    // GPS stuff
    bn357_parse_data(); // Try to parse gps
    got_gps = bn357_get_status_up_to_date(1);

    if(got_gps){
        // ========================================== Gather data form gps
        float latitude_sign = 1 * (bn357_get_latitude_direction() == 'N') + (-1) * (bn357_get_latitude_direction() == 'S'); // Possible results: 0, 1 or -1
        float longitude_sign = 1 * (bn357_get_longitude_direction() == 'E') + (-1) * (bn357_get_longitude_direction() == 'W');

        //Add turn the lat and lon positive based on the sign
        gps_latitude = bn357_get_latitude() * latitude_sign;
        gps_longitude = bn357_get_longitude() * longitude_sign;

        gps_fix_type = bn357_get_fix_type();
        gps_satellites_count = bn357_get_satellites_quantity();
        gps_position_accuracy = bn357_get_accuracy();
        gps_yaw = bn357_get_course_over_ground();
        gps_speed_ms = bn357_get_speed_over_ground_ms();

        // ========================================== Calculate based on data

        // This is basically proportional error but with proper earth diameter calculations
        float dlat = (gps_latitude - target_latitude) * M_DEG_TO_RAD;
        float dlon = (gps_longitude - target_longitude) * M_DEG_TO_RAD;
        float lat_mid = ((gps_latitude + target_latitude) * 0.5f) * M_DEG_TO_RAD;

        lat_distance_to_target_meters = R_EARTH * dlat;
        lon_distance_to_target_meters = R_EARTH * dlon * cosf(lat_mid);

        // Based on gps data get the declination 
        if(!wmm_elements_computed && wmm_perform_elements_compute && gps_satellites_count > 8){
            gps_height_above_geoid_kilometers = bn357_get_geoid_altitude_meters() / 1000.0f;
            gps_date_day = bn357_get_date_day();
            gps_date_month = bn357_get_date_month();
            gps_date_year = bn357_get_date_full_year();
            // wmm_compute_elements(gps_latitude, gps_longitude, gps_height_above_geoid_kilometers, gps_date_year, gps_date_month, gps_date_day);
            // yaw_declination = wmm_get_declination_degrees();
            wmm_elements_computed = 1;
            // TODO add logic that checks how far away last declination point was and recompute the declination
        }

        if(!bn357_get_course_over_ground_stale() && !bn357_get_speed_over_ground_ms_stale()){
            float gps_yaw_rad = gps_yaw * M_DEG_TO_RAD;
            gps_speed_ms_north = gps_speed_ms * cosf(gps_yaw_rad);
            gps_speed_ms_east = gps_speed_ms * sinf(gps_yaw_rad);
        }else{
            gps_speed_ms_north = 0.0f;
            gps_speed_ms_north = 0.0f;
        }

        const float pos_to_vel_scale = 1.0f;  // just a scalar, like a unit conversion
        const float vel_sp_max       = gps_pid_angle_of_attack_max;  // clamp, also just a constant

        float vel_sp_north_raw = pos_to_vel_scale * lat_distance_to_target_meters;
        float vel_sp_east_raw  = pos_to_vel_scale * lon_distance_to_target_meters;

        float vel_sp_mag = sqrtf(vel_sp_north_raw * vel_sp_north_raw +
                                vel_sp_east_raw  * vel_sp_east_raw);

        if (vel_sp_mag > vel_sp_max && vel_sp_mag > 0.001f) {
            float scale = vel_sp_max / vel_sp_mag;
            vel_sp_north = vel_sp_north_raw * scale;
            vel_sp_east  = vel_sp_east_raw  * scale;
        } else {
            vel_sp_north = vel_sp_north_raw;
            vel_sp_east  = vel_sp_east_raw;
        }

        // Velocity error in earth frame (N/E)
        vel_error_east  = vel_sp_east + gps_speed_ms_east;
        vel_error_north = vel_sp_north + gps_speed_ms_north;

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Turn led ON
        last_got_gps_timestamp = get_absolute_time()/1000;
        gps_can_be_used = 1;
    }else if(get_absolute_time()/1000 - last_got_gps_timestamp > max_allowed_time_miliseconds_between_got_gps){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Turn led OFF
        gps_can_be_used = 0;
    }

    // --------------------------------------------------------------------------------------------------- Make adjustments to sensor data to fit orientation of drone 

    invert_axies(acceleration_data);
    invert_axies(gyro_angular);

    // Save old data
    acceleration_data_raw[0] = acceleration_data[0];
    acceleration_data_raw[1] = acceleration_data[1];
    acceleration_data_raw[2] = acceleration_data[2];

    gyro_angular_raw[0] = gyro_angular[0];
    gyro_angular_raw[1] = gyro_angular[1];
    gyro_angular_raw[2] = gyro_angular[2];

    magnetometer_data_raw[0] = magnetometer_data[0];
    magnetometer_data_raw[1] = magnetometer_data[1];
    magnetometer_data_raw[2] = magnetometer_data[2];

    magnetometer_data_secondary_raw[0] = magnetometer_data_secondary[0];
    magnetometer_data_secondary_raw[1] = magnetometer_data_secondary[1];
    magnetometer_data_secondary_raw[2] = magnetometer_data_secondary[2];


    if(txt_logging_mode == 6 || txt_logging_mode == 7){

        altitude_barometer_raw = altitude_barometer;
    }
    // ------------------------------------------------------------------------------------------------------ Filter the sensor data (sensitive to loop HZ disruptions)

    handle_filtering_of_sensor_data();

    // ------------------------------------------------------------------------------------------------------ Sensor fusion
    calculate_roll_pitch_from_accelerometer_data(acceleration_data, &accelerometer_roll, &accelerometer_pitch, accelerometer_roll_offset, accelerometer_pitch_offset);
    sensor_fusion_roll_pitch(gyro_angular, accelerometer_roll, accelerometer_pitch, loop_start_timestamp_microseconds, 1, imu_orientation);

    // Apply mag motor frequency comensation 
    if(use_compass_rpm){
        average_rpm = (motor_frequency[0]+motor_frequency[1]+motor_frequency[2]+motor_frequency[3])/4.0f;

        mag1_x_offset = (-1) * calculate_mag_offset_using_compass_rpm(average_rpm, compass_frequency_frequency_samples, compass_frequency_mag1_x_offsets, compass_frequency_values_amount);
        mag1_y_offset = (-1) * calculate_mag_offset_using_compass_rpm(average_rpm, compass_frequency_frequency_samples, compass_frequency_mag1_y_offsets, compass_frequency_values_amount);
        mag1_z_offset = (-1) * calculate_mag_offset_using_compass_rpm(average_rpm, compass_frequency_frequency_samples, compass_frequency_mag1_z_offsets, compass_frequency_values_amount);

        mag2_x_offset = (-1) * calculate_mag_offset_using_compass_rpm(average_rpm, compass_frequency_frequency_samples, compass_frequency_mag2_x_offsets, compass_frequency_values_amount);
        mag2_y_offset = (-1) * calculate_mag_offset_using_compass_rpm(average_rpm, compass_frequency_frequency_samples, compass_frequency_mag2_y_offsets, compass_frequency_values_amount);
        mag2_z_offset = (-1) * calculate_mag_offset_using_compass_rpm(average_rpm, compass_frequency_frequency_samples, compass_frequency_mag2_z_offsets, compass_frequency_values_amount);

        magnetometer_data[0] = magnetometer_data[0] + mag1_x_offset;
        magnetometer_data[1] = magnetometer_data[1] + mag1_y_offset;
        magnetometer_data[2] = magnetometer_data[2] + mag1_z_offset;

        magnetometer_data_secondary[0] = magnetometer_data_secondary[0] + mag2_x_offset;
        magnetometer_data_secondary[1] = magnetometer_data_secondary[1] + mag2_y_offset;
        magnetometer_data_secondary[2] = magnetometer_data_secondary[2] + mag2_z_offset;
    }


    // Calculat yaw
    calculate_yaw_tilt_compensated_using_magnetometer_data(
        magnetometer_data, 
        &magnetometer_yaw, 
        imu_orientation[0] - accelerometer_roll_offset, // Dont use offset roll and pitch, has to be pure
        imu_orientation[1] - accelerometer_pitch_offset,
        yaw_declination,
        0
    );

    calculate_yaw_tilt_compensated_using_magnetometer_data(
        magnetometer_data_secondary, 
        &magnetometer_yaw_secondary, 
        imu_orientation[0] - accelerometer_roll_offset, // Dont use offset roll and pitch, has to be pure
        imu_orientation[1] - accelerometer_pitch_offset,
        yaw_declination,
        0
    );

    get_gyro_yaw(gyro_angular, loop_start_timestamp_microseconds, 1, &gyro_yaw);
    get_gyro_yaw2(gyro_angular, loop_start_timestamp_microseconds, 1, &gyro_yaw2, imu_orientation);

    if(initial_yaw_set == 0 && get_absolute_time() - yaw_loop_start_timestamp > yaw_set_yaw_after_microseconds_of_loop){
        initial_yaw_set = 1;
        gyro_yaw = magnetometer_yaw_secondary;
        gyro_yaw2 = magnetometer_yaw_secondary;
        imu_orientation[2] = magnetometer_yaw_secondary;
    }

    sensor_fusion_yaw(magnetometer_yaw_secondary, gyro_angular, loop_start_timestamp_microseconds, 1, imu_orientation);

    if(txt_logging_mode == 4){

        calculate_yaw_tilt_compensated_using_magnetometer_data(
            magnetometer_data_unrotated, 
            &magnetometer_yaw_unrotated, 
            imu_orientation[0], // Dont use offset roll and pitch, has to be pure
            imu_orientation[1],
            yaw_declination - 180.0f,
            1
        );
        
        calculate_yaw_tilt_compensated_using_magnetometer_data(
            magnetometer_data, 
            &magnetometer_yaw_90, 
            -imu_orientation[1] - accelerometer_pitch_offset, // Dont use offset roll and pitch, has to be pure
            imu_orientation[0] - accelerometer_roll_offset,
            yaw_declination,
            1
        );

        calculate_yaw_tilt_compensated_using_magnetometer_data(
            magnetometer_data, 
            &magnetometer_yaw_180, 
            -(imu_orientation[0] - accelerometer_roll_offset), // Dont use offset roll and pitch, has to be pure
            -(imu_orientation[1] - accelerometer_pitch_offset),
            yaw_declination,
            1
        );

        calculate_yaw_tilt_compensated_using_magnetometer_data(
            magnetometer_data, 
            &magnetometer_yaw_270, 
            imu_orientation[1] - accelerometer_pitch_offset, // Dont use offset roll and pitch, has to be pure
            -(imu_orientation[0] - accelerometer_roll_offset),
            yaw_declination,
            1
        );

        calculate_yaw_tilt_compensated_using_magnetometer_data(
            magnetometer_data_secondary_unrotated, 
            &magnetometer_yaw_secondary_unrotated, 
            -imu_orientation[1], // Dont use offset roll and pitch, has to be pure
            imu_orientation[0],
            yaw_declination - 90.0f,
            0
        );

    }
    
    // GPS STUFF
    float yaw_rad = low_pass_filter_pt1_read(&lowpass_yaw_rad, imu_orientation[2] * M_DEG_TO_RAD);

    // This is just a rotation matrix in linear algebra rotate coordinates by degrees
    error_gps_roll = -(-vel_error_north * sinf(yaw_rad) + vel_error_east * cosf(yaw_rad));
    error_gps_pitch =  -(vel_error_north * cosf(yaw_rad) + vel_error_east * sinf(yaw_rad));

    // --------------------- TODO DO SOME CROSS AXIS CORRECTION (Gyro gimbal Lock/Transfer) for roll and pitch

    // ------------------------------------------------------------------------------------------------------ Use sensor fused data for more data
    vertical_acceleration_old = old_mpu6050_calculate_vertical_acceleration_cm_per_second(acceleration_data, imu_orientation);
    vertical_acceleration = mpu6050_calculate_vertical_acceleration_cm_per_second(acceleration_data, imu_orientation);

    // kalman_filter_predict(&altitude_and_velocity_kalman, &vertical_acceleration);
    // kalman_filter_update(&altitude_and_velocity_kalman, &altitude_barometer);

    // altitude_sensor_fusion = kalman_filter_get_state(&altitude_and_velocity_kalman)[0][0];
    // vertical_velocity = kalman_filter_get_state(&altitude_and_velocity_kalman)[1][0];

    // printf("Alt %f vel %f ", altitude_sensor_fusion, vertical_velocity);
    // printf("IMU %f %f %f\n", imu_orientation[0], imu_orientation[1], imu_orientation[2]);

    // printf("Lat %f, Lon %f\n", gps_latitude, gps_longitude);
    // printf("%f;%f;%f;\n", mangetometer_yaw_unfiltered, mangetometer_yaw_low_pass, magnetometer_yaw);

}

void handle_filtering_of_sensor_data(){

    // Max 1.79. Without 1.46
    // RPM filtering
    if(motor_frequency[0] >= rpm_notch_filter_min_frequency){

        // 1st harmonic
        if(motor_frequency[0] < nyquist_with_margin &&
            motor_frequency[1] < nyquist_with_margin &&
            motor_frequency[2] < nyquist_with_margin &&
            motor_frequency[3] < nyquist_with_margin
        ){
            notch_filter_q_set_center_frequency(&gyro_x_notch_filter_harmonic_1_motor_0, motor_frequency[0]);
            notch_filter_q_set_center_frequency(&gyro_x_notch_filter_harmonic_1_motor_1, motor_frequency[1]);
            notch_filter_q_set_center_frequency(&gyro_x_notch_filter_harmonic_1_motor_2, motor_frequency[2]);
            notch_filter_q_set_center_frequency(&gyro_x_notch_filter_harmonic_1_motor_3, motor_frequency[3]);
            notch_filter_q_set_center_frequency_using_reference_filter(&gyro_y_notch_filter_harmonic_1_motor_0, &gyro_x_notch_filter_harmonic_1_motor_0);
            notch_filter_q_set_center_frequency_using_reference_filter(&gyro_y_notch_filter_harmonic_1_motor_1, &gyro_x_notch_filter_harmonic_1_motor_1);
            notch_filter_q_set_center_frequency_using_reference_filter(&gyro_y_notch_filter_harmonic_1_motor_2, &gyro_x_notch_filter_harmonic_1_motor_2);
            notch_filter_q_set_center_frequency_using_reference_filter(&gyro_y_notch_filter_harmonic_1_motor_3, &gyro_x_notch_filter_harmonic_1_motor_3);

            gyro_angular[0] = notch_filter_q_read(&gyro_x_notch_filter_harmonic_1_motor_0, gyro_angular[0]);
            gyro_angular[0] = notch_filter_q_read(&gyro_x_notch_filter_harmonic_1_motor_1, gyro_angular[0]);
            gyro_angular[0] = notch_filter_q_read(&gyro_x_notch_filter_harmonic_1_motor_2, gyro_angular[0]);
            gyro_angular[0] = notch_filter_q_read(&gyro_x_notch_filter_harmonic_1_motor_3, gyro_angular[0]);

            gyro_angular[1] = notch_filter_q_read(&gyro_y_notch_filter_harmonic_1_motor_0, gyro_angular[1]);
            gyro_angular[1] = notch_filter_q_read(&gyro_y_notch_filter_harmonic_1_motor_1, gyro_angular[1]);
            gyro_angular[1] = notch_filter_q_read(&gyro_y_notch_filter_harmonic_1_motor_2, gyro_angular[1]);
            gyro_angular[1] = notch_filter_q_read(&gyro_y_notch_filter_harmonic_1_motor_3, gyro_angular[1]);
        }


        // 3nd harmonic
        if(motor_frequency[0] * 3 < nyquist_with_margin &&
            motor_frequency[1] * 3 < nyquist_with_margin &&
            motor_frequency[2] * 3 < nyquist_with_margin &&
            motor_frequency[3] * 3 < nyquist_with_margin
        ){
            notch_filter_q_set_center_frequency(&gyro_x_notch_filter_harmonic_3_motor_0, motor_frequency[0] * 3);
            notch_filter_q_set_center_frequency(&gyro_x_notch_filter_harmonic_3_motor_1, motor_frequency[1] * 3);
            notch_filter_q_set_center_frequency(&gyro_x_notch_filter_harmonic_3_motor_2, motor_frequency[2] * 3);
            notch_filter_q_set_center_frequency(&gyro_x_notch_filter_harmonic_3_motor_3, motor_frequency[3] * 3);
            notch_filter_q_set_center_frequency_using_reference_filter(&gyro_y_notch_filter_harmonic_3_motor_0, &gyro_x_notch_filter_harmonic_3_motor_0);
            notch_filter_q_set_center_frequency_using_reference_filter(&gyro_y_notch_filter_harmonic_3_motor_1, &gyro_x_notch_filter_harmonic_3_motor_1);
            notch_filter_q_set_center_frequency_using_reference_filter(&gyro_y_notch_filter_harmonic_3_motor_2, &gyro_x_notch_filter_harmonic_3_motor_2);
            notch_filter_q_set_center_frequency_using_reference_filter(&gyro_y_notch_filter_harmonic_3_motor_3, &gyro_x_notch_filter_harmonic_3_motor_3);

            gyro_angular[0] = notch_filter_q_read(&gyro_x_notch_filter_harmonic_3_motor_0, gyro_angular[0]);
            gyro_angular[0] = notch_filter_q_read(&gyro_x_notch_filter_harmonic_3_motor_1, gyro_angular[0]);
            gyro_angular[0] = notch_filter_q_read(&gyro_x_notch_filter_harmonic_3_motor_2, gyro_angular[0]);
            gyro_angular[0] = notch_filter_q_read(&gyro_x_notch_filter_harmonic_3_motor_3, gyro_angular[0]);

            gyro_angular[1] = notch_filter_q_read(&gyro_y_notch_filter_harmonic_3_motor_0, gyro_angular[1]);
            gyro_angular[1] = notch_filter_q_read(&gyro_y_notch_filter_harmonic_3_motor_1, gyro_angular[1]);
            gyro_angular[1] = notch_filter_q_read(&gyro_y_notch_filter_harmonic_3_motor_2, gyro_angular[1]);
            gyro_angular[1] = notch_filter_q_read(&gyro_y_notch_filter_harmonic_3_motor_3, gyro_angular[1]);
        }

        // If the frequencies are above nyquist with margin then filtering is not done
        // If it would be done, the filters would start oscillating the data and work against it at worst
        // or just introduce stronger noise 

    }

    // Dynamic notch filtering
    // Not currently used

    // For testing
    if(txt_logging_mode == 4){
        // Keep some unfiltered data
        magnetometer_data_unfiltered[0] = magnetometer_data[0];
        magnetometer_data_unfiltered[1] = magnetometer_data[1];
        magnetometer_data_unfiltered[2] = magnetometer_data[2];
        
        magnetometer_data_ist8310_unfiltered[0] = magnetometer_data_ist8310[0];
        magnetometer_data_ist8310_unfiltered[1] = magnetometer_data_ist8310[1];
        magnetometer_data_ist8310_unfiltered[2] = magnetometer_data_ist8310[2];

        
        if(got_ist8310_reading){
            magnetometer_data_ist8310[0] = low_pass_filter_biquad_read(&filter_magnetometer_ist8310_x, magnetometer_data_ist8310[0]);
            magnetometer_data_ist8310[1] = low_pass_filter_biquad_read(&filter_magnetometer_ist8310_y, magnetometer_data_ist8310[1]);
            magnetometer_data_ist8310[2] = low_pass_filter_biquad_read(&filter_magnetometer_ist8310_z, magnetometer_data_ist8310[2]);
        }
    }

    // Filter magnetometer data

    magnetometer_data[0] = low_pass_filter_biquad_read(&filter_magnetometer_x, magnetometer_data[0]);
    magnetometer_data[1] = low_pass_filter_biquad_read(&filter_magnetometer_y, magnetometer_data[1]);
    magnetometer_data[2] = low_pass_filter_biquad_read(&filter_magnetometer_z, magnetometer_data[2]);

    magnetometer_data_secondary[0] = low_pass_filter_biquad_read(&filter_magnetometer_x_secondary, magnetometer_data_secondary[0]);
    magnetometer_data_secondary[1] = low_pass_filter_biquad_read(&filter_magnetometer_y_secondary, magnetometer_data_secondary[1]);
    magnetometer_data_secondary[2] = low_pass_filter_biquad_read(&filter_magnetometer_z_secondary, magnetometer_data_secondary[2]);

    magnetometer_data_unrotated[0] = low_pass_filter_biquad_read(&filter_magnetometer_90_x, magnetometer_data_unrotated[0]);
    magnetometer_data_unrotated[1] = low_pass_filter_biquad_read(&filter_magnetometer_90_y, magnetometer_data_unrotated[1]);
    magnetometer_data_unrotated[2] = low_pass_filter_biquad_read(&filter_magnetometer_90_z, magnetometer_data_unrotated[2]);

    magnetometer_data_secondary_unrotated[0] = low_pass_filter_biquad_read(&filter_magnetometer_180_x, magnetometer_data_secondary_unrotated[0]);
    magnetometer_data_secondary_unrotated[1] = low_pass_filter_biquad_read(&filter_magnetometer_180_y, magnetometer_data_secondary_unrotated[1]);
    magnetometer_data_secondary_unrotated[2] = low_pass_filter_biquad_read(&filter_magnetometer_180_z, magnetometer_data_secondary_unrotated[2]);

    magnetometer_data_raw[0] = magnetometer_data[0];
    magnetometer_data_raw[1] = magnetometer_data[1];
    magnetometer_data_raw[2] = magnetometer_data[2];

    magnetometer_data_secondary_raw[0] = magnetometer_data_secondary[0];
    magnetometer_data_secondary_raw[1] = magnetometer_data_secondary[1];
    magnetometer_data_secondary_raw[2] = magnetometer_data_secondary[2];

    // Yaw Angular rotation filtering
    gyro_angular[2] = low_pass_filter_pt1_read(&filter_gyro_z, gyro_angular[2]);

    // Filter accelerometer data
    acceleration_data[0] = low_pass_filter_biquad_read(&biquad_filter_accelerometer_x, acceleration_data[0]);
    acceleration_data[1] = low_pass_filter_biquad_read(&biquad_filter_accelerometer_y, acceleration_data[1]);
    acceleration_data[2] = low_pass_filter_biquad_read(&biquad_filter_accelerometer_z, acceleration_data[2]);

    // Additional gyro filtering for d term
    // Adjust the filter cutoff 
    low_pass_filter_biquad_set_cutoff_frequency(
        &roll_d_term_filtering, 
        map_value_to_expo_range_inverted(
            throttle, 
            d_term_filtering_min_cutoff,
            d_term_filtering_max_cutoff,
            d_term_filtering_expo
        )
    ); // Use throttle, not the final output of the motor
    low_pass_filter_biquad_set_cutoff_frequency_using_reference_filter(&pitch_d_term_filtering, &roll_d_term_filtering);
    gyro_angular_for_d_term[0] = low_pass_filter_biquad_read(&roll_d_term_filtering, gyro_angular[0]);
    gyro_angular_for_d_term[1] = low_pass_filter_biquad_read(&pitch_d_term_filtering, gyro_angular[1]);

    altitude_barometer = low_pass_filter_biquad_read(&altitude_barometer_filtering, altitude_barometer);

}

float map_value_to_expo_range(float value, float min_expo, float max_expo, uint8_t expo_curve){
    // Assuming value is from 0 to 100
    // Will divide value by 100 to get a value from 0 to 1
    return min_expo + powf(value* 0.01, expo_curve) * (max_expo - min_expo);
}

float map_value_to_expo_range_inverted(float value, float min_expo, float max_expo, uint8_t expo_curve){
    // Assuming value is from 0 to 100
    // Will divide value by 100 to get a value from 0 to 1
    // Some extra logic in needed to invert it.
    return min_expo + (1.0f - powf(1.0f - (value* 0.01), expo_curve)) * (max_expo - min_expo);
}


void reset_array_data(float *array, uint8_t array_size){
    memset(array, 0, array_size * sizeof(float));
}

void invert_axies(float *data){
    // This effectively turns the sensor around 180 degrees on z axis
    data[0] = -data[0];
    data[1] = -data[1];
}


void flip_sensor_data_upside_down(float *data){
    // This effectively flips the sensor around 180 degrees on x or y axis. Basically upside down

    float temp[3] = {0.0f, 0.0f, 0.0f};

    temp[0] = -data[1];
    temp[1] = -data[0];
    temp[2] = -data[2];
}