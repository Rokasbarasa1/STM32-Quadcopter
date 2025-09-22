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
    mmc5603_magnetometer_readings_micro_teslas(magnetometer_data_secondary);
    bmm350_magnetometer_readings_micro_teslas(magnetometer_data, 1);
    // qmc5883l_magnetometer_readings_micro_teslas(magnetometer_data);
    // hmc5883l_magnetometer_readings_micro_teslas(magnetometer_data);


    sensors3 = DWT->CYCCNT;
    mpu6050_get_accelerometer_readings_gravity(acceleration_data);
    sensors4 = DWT->CYCCNT;
    mpu6050_get_gyro_readings_dps(gyro_angular);
    __enable_irq();
    sensors5 = DWT->CYCCNT;

    // printf("ACCEL,%5.2f,%5.2f,%5.2f;", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
    // printf("GYRO,%5.2f,%5.2f,%5.2f;\n", gyro_angular[0], gyro_angular[1], gyro_angular[2]);

    // GPS stuff
    bn357_parse_data(); // Try to parse gps
    got_gps = bn357_get_status_up_to_date(1);

    if(got_gps){
        float latitude_sign = 1 * (bn357_get_latitude_direction() == 'N') + (-1) * (bn357_get_latitude_direction() == 'S'); // Possible results: 0, 1 or -1
        float longitude_sign = 1 * (bn357_get_longitude_direction() == 'E') + (-1) * (bn357_get_longitude_direction() == 'W');

        //Add turn the lat and lon positive based on the sign
        gps_latitude = bn357_get_latitude() * latitude_sign;
        gps_longitude = bn357_get_longitude() * longitude_sign;

        gps_fix_type = bn357_get_fix_type();
        gps_satellites_count = bn357_get_satellites_quantity();
        gps_yaw = bn357_get_course_over_ground();

        // This is basically proportional error
        lat_distance_to_target_meters = (gps_latitude - target_latitude) * 111320.0f;
        lon_distance_to_target_meters = (gps_longitude - target_longitude) * 111320.0f * cosf(gps_latitude * M_DEG_TO_RAD);

        if(
            lat_distance_to_target_meters > 100.0f || 
            lat_distance_to_target_meters < -100.0f || 
            lon_distance_to_target_meters > 100.0f || 
            lon_distance_to_target_meters < -100.0f
        ){
            // Somewhere in this biquad filter it divides by zero and fucks everything up
            // low_pass_filter_biquad_set_initial_values(&biquad_filter_gps_lat, lat_distance_to_target_meters);
            // low_pass_filter_biquad_set_initial_values(&biquad_filter_gps_lon, lon_distance_to_target_meters);

            
            low_pass_filter_pt1_set_initial_values(&biquad_filter_gps_lat, lat_distance_to_target_meters);
            low_pass_filter_pt1_set_initial_values(&biquad_filter_gps_lon, lon_distance_to_target_meters);

        }else{
            // lat_distance_to_target_meters = low_pass_filter_biquad_read(&biquad_filter_gps_lat, lat_distance_to_target_meters);
            // lon_distance_to_target_meters = low_pass_filter_biquad_read(&biquad_filter_gps_lon, lon_distance_to_target_meters);

            lat_distance_to_target_meters = low_pass_filter_pt1_read(&biquad_filter_gps_lat, lat_distance_to_target_meters);
            lon_distance_to_target_meters = low_pass_filter_pt1_read(&biquad_filter_gps_lon, lon_distance_to_target_meters);
        }

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        last_got_gps_timestamp = get_absolute_time()/1000;
        gps_can_be_used = 1;
    }else if(get_absolute_time()/1000 - last_got_gps_timestamp > max_allowed_time_miliseconds_between_got_gps){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        gps_can_be_used = 0;
    }

    // --------------------------------------------------------------------------------------------------- Make adjustments to sensor data to fit orientation of drone 
    // Specifically for bmm350
    magnetometer_data[1] = magnetometer_data[1] * -1;

    // IST8310
    // if(got_ist8310_reading){
    //     rotate_magnetometer_output_90_degrees_anti_clockwise(magnetometer_data_ist8310);
    // }

    // Pitch (+)forwards - front of device nose goes down
    // Roll (+)right - the device banks to the right side while pointing forward
    // After yaw calculation yaw has to be 0/360 when facing north in the pitch+ axis
    // After getting roll and pitch from accelerometer. Pitch forwards -> +Pitch. Roll right -> +Roll

    // rotate_magnetometer_output_90_degrees_anti_clockwise(magnetometer_data);
    invert_axies(acceleration_data);
    invert_axies(gyro_angular);
    // Pitch device forwards -> Y axis positive. Roll device right -> X axis positive

    // magnetometer_data_ist8310_raw[0] = magnetometer_data_ist8310[0];
    // magnetometer_data_ist8310_raw[1] = magnetometer_data_ist8310[1];
    // magnetometer_data_ist8310_raw[2] = magnetometer_data_ist8310[2];

    magnetometer_data_raw[0] = magnetometer_data[0];
    magnetometer_data_raw[1] = magnetometer_data[1];
    magnetometer_data_raw[2] = magnetometer_data[2];

    magnetometer_data_secondary_raw[0] = magnetometer_data_secondary[0];
    magnetometer_data_secondary_raw[1] = magnetometer_data_secondary[1];
    magnetometer_data_secondary_raw[2] = magnetometer_data_secondary[2];

    if(txt_logging_mode == 6 || txt_logging_mode == 7){
        gyro_angular_raw[0] = gyro_angular[0];
        gyro_angular_raw[1] = gyro_angular[1];
        gyro_angular_raw[2] = gyro_angular[2];

        acceleration_data_raw[0] = acceleration_data[0];
        acceleration_data_raw[1] = acceleration_data[1];
        acceleration_data_raw[2] = acceleration_data[2];

        magnetometer_data_raw[0] = magnetometer_data[0];
        magnetometer_data_raw[1] = magnetometer_data[1];
        magnetometer_data_raw[2] = magnetometer_data[2];

        altitude_barometer_raw = altitude_barometer;
    }
    // ------------------------------------------------------------------------------------------------------ Filter the sensor data (sensitive to loop HZ disruptions)
    sensors6 = DWT->CYCCNT;

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
    sensors7 = DWT->CYCCNT;

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


    magnetometer_data_raw[0] = magnetometer_data[0];
    magnetometer_data_raw[1] = magnetometer_data[1];
    magnetometer_data_raw[2] = magnetometer_data[2];

    magnetometer_data_secondary_raw[0] = magnetometer_data_secondary[0];
    magnetometer_data_secondary_raw[1] = magnetometer_data_secondary[1];
    magnetometer_data_secondary_raw[2] = magnetometer_data_secondary[2];
    // magnetometer_data[0] = outlier_detection_process(&outlier_detection_magnetometer_x, magnetometer_data[0]);
    // magnetometer_data[1] = outlier_detection_process(&outlier_detection_magnetometer_y, magnetometer_data[1]);
    // magnetometer_data[2] = outlier_detection_process(&outlier_detection_magnetometer_z, magnetometer_data[2]);

    // magnetometer_data[0] = iir_filter_process(&iir_filter_magnetometer_x, magnetometer_data[0]);
    // magnetometer_data[1] = iir_filter_process(&iir_filter_magnetometer_y, magnetometer_data[1]);
    // magnetometer_data[2] = iir_filter_process(&iir_filter_magnetometer_z, magnetometer_data[2]);

    // magnetometer_data[0] = moving_average_process(&moving_average_magnetometer_x, magnetometer_data[0]);
    // magnetometer_data[1] = moving_average_process(&moving_average_magnetometer_y, magnetometer_data[1]);
    // magnetometer_data[2] = moving_average_process(&moving_average_magnetometer_z, magnetometer_data[2]);




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

    sensors8 = DWT->CYCCNT;

    altitude_barometer = low_pass_filter_biquad_read(&altitude_barometer_filtering, altitude_barometer);

    // ------------------------------------------------------------------------------------------------------ Sensor fusion
    calculate_roll_pitch_from_accelerometer_data(acceleration_data, &accelerometer_roll, &accelerometer_pitch, accelerometer_roll_offset, accelerometer_pitch_offset); // Get roll and pitch from the data. I call it x and y. Ranges -90 to 90. 
    sensor_fusion_roll_pitch(gyro_angular, accelerometer_roll, accelerometer_pitch, loop_start_timestamp_microseconds, 1, imu_orientation);

    // Main objectives
    // Rotation
    // Tilt compensation
    // FIltering
    // DIfferent sensors


    // Ttime
    // Gyro yaw
    get_gyro_yaw(gyro_angular, loop_start_timestamp_microseconds, 1, &gyro_yaw);


    // Apply mag offsets


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




    // X bmm350 yaw rotated tilt
    calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data, &magnetometer_yaw, imu_orientation[0], imu_orientation[1], yaw_offset + yaw_declination);
    calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data_secondary, &magnetometer_yaw_secondary, -imu_orientation[1], imu_orientation[0], yaw_offset + 90.0  + yaw_declination);


    if(txt_logging_mode == 4){
        // magnetometer_rotation_matrix

        // float magnetometer_data_matrix_rotate[3];
        // float magnetometer_data_matrix_rotate1[3];
        // float magnetometer_data_matrix_rotate2[3];

        // magnetometer_data_matrix_rotate[0] = magnetometer_data_secondary[0];
        // magnetometer_data_matrix_rotate[1] = magnetometer_data_secondary[1];
        // magnetometer_data_matrix_rotate[2] = magnetometer_data_secondary[2];

        // magnetometer_data_matrix_rotate1[0] = magnetometer_data_secondary[0];
        // magnetometer_data_matrix_rotate1[1] = magnetometer_data_secondary[1];
        // magnetometer_data_matrix_rotate1[2] = magnetometer_data_secondary[2];

        // magnetometer_data_matrix_rotate2[0] = magnetometer_data_secondary[0];
        // magnetometer_data_matrix_rotate2[1] = magnetometer_data_secondary[1];
        // magnetometer_data_matrix_rotate2[2] = magnetometer_data_secondary[2];


        // rotate_magnetometer_inplace(magnetometer_rotation_matrix, magnetometer_data_matrix_rotate);
        // rotate_magnetometer_inplace(magnetometer_rotation_matrix1, magnetometer_data_matrix_rotate1);
        // rotate_magnetometer_inplace(magnetometer_rotation_matrix2, magnetometer_data_matrix_rotate2);


        // bmm350 yaw unrotated no tilt
        calculate_yaw_using_magnetometer_data(magnetometer_data, &magnetometer_yaw_unrotated_no_tilt, 0.0f);

        // bmm350 yaw unrotated tilt
        calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data, &magnetometer_yaw_unrotated_tilt, imu_orientation[0], imu_orientation[1], 0.0f);

        // bmm350 yaw rotated no tilt
        calculate_yaw_using_magnetometer_data(magnetometer_data, &magnetometer_yaw_no_tilt, yaw_offset + yaw_declination);

        
        // Roll and pitch rotated 90 degrees
        // calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data_matrix_rotate, &magnetometer_yaw_90, imu_orientation[0], imu_orientation[1], yaw_offset + yaw_declination);
        // calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data_matrix_rotate1, &magnetometer_yaw_180, imu_orientation[0], imu_orientation[1], yaw_offset + yaw_declination);
        // calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data_matrix_rotate2, &magnetometer_yaw_270, imu_orientation[0], imu_orientation[1], yaw_offset + yaw_declination);

        calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data, &magnetometer_yaw_90, -imu_orientation[1], imu_orientation[0], yaw_offset + yaw_declination);
        calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data, &magnetometer_yaw_180, -imu_orientation[0], -imu_orientation[1], yaw_offset + yaw_declination);
        calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data, &magnetometer_yaw_270, imu_orientation[1], -imu_orientation[0], yaw_offset + yaw_declination);
        
        // calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data_matrix_rotate, &magnetometer_yaw_90, -imu_orientation[1], imu_orientation[0], yaw_offset + 90.0 + yaw_declination);
        // calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data_matrix_rotate1, &magnetometer_yaw_180, -imu_orientation[1], imu_orientation[0], yaw_offset + 90.0 + yaw_declination);
        // calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data_matrix_rotate2, &magnetometer_yaw_270, -imu_orientation[1], imu_orientation[0], yaw_offset + 90.0 + yaw_declination);

        
        // bmm350 yaw rotated unfiltered no tilt
        calculate_yaw_using_magnetometer_data(magnetometer_data_unfiltered, &magnetometer_yaw_unfiltered_no_tilt, yaw_offset + yaw_declination);


        // ist8310 yaw unrotated no tilt
        calculate_yaw_using_magnetometer_data(magnetometer_data_ist8310, &magnetometer_ist8310_yaw_unrotated_no_tilt, 0.0f);

        // ist8310 yaw unrotated tilt
        calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data_ist8310, &magnetometer_ist8310_yaw_unrotated_tilt, imu_orientation[0], imu_orientation[1], 0.0f);

        // ist8310 yaw rotated no tilt
        calculate_yaw_using_magnetometer_data(magnetometer_data_ist8310, &magnetometer_ist8310_yaw_no_tilt, yaw_offset + yaw_declination);

        // ist8310 yaw rotated tilt
        calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data_ist8310, &magnetometer_ist8310_yaw, imu_orientation[0], imu_orientation[1], yaw_offset + yaw_declination);

        // ist8310 yaw rotated unfiltered no tilt
        calculate_yaw_using_magnetometer_data(magnetometer_data_ist8310_unfiltered, &magnetometer_ist8310_yaw_unfiltered_no_tilt, yaw_offset + yaw_declination);
    }


    // // Set new yaw after some time
    // if(initial_yaw_set == 0 && get_absolute_time() - yaw_loop_start_timestamp > yaw_set_yaw_after_microseconds_of_loop){
    //     initial_yaw_set = 1;
    //     imu_orientation[2] = magnetometer_yaw;
    //     gyro_yaw = magnetometer_yaw;
    //     gyro_yaw_old = magnetometer_yaw_old;
    // }

    // acceleration_data_previous[0] = acceleration_data[0];
    // acceleration_data_previous[1] = acceleration_data[1];
    // acceleration_data_previous[2] = acceleration_data[2];

    // if (!initial_yaw_set) {
    //     // float gyro_magnitude = sqrtf(
    //     //     gyro_angular[0] * gyro_angular[0] +
    //     //     gyro_angular[1] * gyro_angular[1] +
    //     //     gyro_angular[2] * gyro_angular[2]
    //     // );

    //     // float accel_change = sqrtf(
    //     //     (acceleration_data[0] - acceleration_data_previous[0]) * (acceleration_data[0] - acceleration_data_previous[0]) +
    //     //     (acceleration_data[1] - acceleration_data_previous[1]) * (acceleration_data[1] - acceleration_data_previous[1]) +
    //     //     (acceleration_data[2] - acceleration_data_previous[2]) * (acceleration_data[2] - acceleration_data_previous[2])
    //     // );

    //     if (fabs(imu_orientation[0]) < 0.2f &&  // roll < 3°
    //         fabs(imu_orientation[1]) < 0.2f)  // pitch < 3°
    //     {
    //     // if (fabs(imu_orientation[0]) < 0.3f &&  // roll < 3°
    //     //     fabs(imu_orientation[1]) < 0.3f &&  // pitch < 3°
    //     //     gyro_magnitude < 0.1f &&            // not rotating
    //     //     accel_change < 0.05f)               // not being bumped
    //     // {
    //         initial_yaw_set = 1;
    //         imu_orientation[2] = magnetometer_yaw;
    //         gyro_yaw = magnetometer_yaw;
    //         gyro_yaw_old = magnetometer_yaw_old;
    //     }else{
    //         gps_target_unset_logged = 0;
    //         gps_target_unset_cause = 5;
    //         gps_can_be_used = 0;
    //     }
    // }


    // WHEN THE USER IS YAWING INCREASE THE MAGNETOMETER COMPONENT. WHEN HE IS NOT KEEP IT LOW LOW LOW
    // if(yaw != 50){
    //     yaw_alpha = COMPLEMENTARY_RATIO_MULTIPLYER_YAW_USER_INPUT_MORE_MORE_MAG;
    //     mpu6050_set_complementary_ratio_yaw(COMPLEMENTARY_RATIO_MULTIPLYER_YAW_USER_INPUT_MORE_MORE_MAG);
    // }else if(fabs(imu_orientation[0]) <= yaw_magnetometer_only_gate_absolute_degrees && fabs(imu_orientation[1]) <= yaw_magnetometer_only_gate_absolute_degrees){
    //     // MAG ONLY
    //     yaw_alpha = COMPLEMENTARY_RATIO_MULTIPLYER_YAW_UPPER_GATE_MORE_MAG;
    //     mpu6050_set_complementary_ratio_yaw(COMPLEMENTARY_RATIO_MULTIPLYER_YAW_UPPER_GATE_MORE_MAG);
    // }else if(fabs(imu_orientation[0]) >= yaw_gyro_only_gate_absolute_degrees && fabs(imu_orientation[1]) >= yaw_gyro_only_gate_absolute_degrees){
    //     yaw_alpha = COMPLEMENTARY_RATIO_MULTIPLYER_YAW_LOWER_GATE_MORE_GYRO;
    //     mpu6050_set_complementary_ratio_yaw(COMPLEMENTARY_RATIO_MULTIPLYER_YAW_LOWER_GATE_MORE_GYRO);
    //     // GYRO ONLY
    // }else{
    //     // SCALE BETWEEN MAG AND GYRO based on highest value of roll or pitch
    //     float max_deviation_from_zero = fmax(fabs(imu_orientation[0]), fabs(imu_orientation[1]));

    //     // map_value_to_expo_range_inverted()
    //     float new_alpha_value = map_value(
    //         max_deviation_from_zero, 
    //         yaw_magnetometer_only_gate_absolute_degrees,
    //         yaw_gyro_only_gate_absolute_degrees,
    //         COMPLEMENTARY_RATIO_MULTIPLYER_YAW_UPPER_GATE_MORE_MAG,
    //         COMPLEMENTARY_RATIO_MULTIPLYER_YAW_LOWER_GATE_MORE_GYRO
    //     );

    //     // Cap the value just in case
    //     if(new_alpha_value < 0.0f){
    //         new_alpha_value = 0.0f;
    //     }else if(new_alpha_value > 1.0f){
    //         new_alpha_value = 1.0f;
    //     }

    //     yaw_alpha = new_alpha_value;

    //     mpu6050_set_complementary_ratio_yaw(new_alpha_value);
    // }
    // sensor_fusion_yaw(gyro_angular, magnetometer_yaw, loop_start_timestamp_microseconds, 1, imu_orientation, &gyro_yaw);

    // Extra imu logging
    // if(txt_logging_mode == 6){
    //     float temp_old_gyro_angular[] = {0.0f,0.0f,0.0f};
    //     temp_old_gyro_angular[0] = (gyro_angular[0] / (1.0f / 65.5f)) * (1.0f / 131.0f);
    //     temp_old_gyro_angular[1] = (gyro_angular[1] / (1.0f / 65.5f)) * (1.0f / 131.0f);
    //     temp_old_gyro_angular[2] = gyro_angular[2];
        
    //     sensor_fusion_roll_pitch(temp_old_gyro_angular, accelerometer_roll, accelerometer_pitch, loop_start_timestamp_microseconds, 0, old_imu_orientation);
    //     // calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data, &magnetometer_yaw_old, old_imu_orientation[0], old_imu_orientation[1], yaw_offset);
    //     // sensor_fusion_yaw(temp_old_gyro_angular, magnetometer_yaw_old, loop_start_timestamp_microseconds, 0, old_imu_orientation, &gyro_yaw_old);
    // }

    // No need for sensor fusion, it introduces delay
    imu_orientation[2] = magnetometer_yaw_secondary;
    
    sensors9 = DWT->CYCCNT;
    // GPS STUFF
    float yaw_rad = (imu_orientation[2] + 0.0f) * M_DEG_TO_RAD;

    // This is just a rotation matrix in linear algebra rotate coordinates by degrees
    error_forward = lat_distance_to_target_meters * cosf(yaw_rad) + lon_distance_to_target_meters * sinf(yaw_rad);
    error_right   = -lat_distance_to_target_meters * sinf(yaw_rad) + lon_distance_to_target_meters * cosf(yaw_rad);


    // --------------------- TODO DO SOME CROSS AXIS CORRECTION (Gyro gimbal Lock/Transfer) for roll and pitch

    sensors10 = DWT->CYCCNT;
    // ------------------------------------------------------------------------------------------------------ Use sensor fused data for more data
    // vertical_acceleration_old = old_mpu6050_calculate_vertical_acceleration_cm_per_second(acceleration_data, imu_orientation);
    // vertical_acceleration = mpu6050_calculate_vertical_acceleration_cm_per_second(acceleration_data, imu_orientation);

    // kalman_filter_predict(&altitude_and_velocity_kalman, &vertical_acceleration);
    // kalman_filter_update(&altitude_and_velocity_kalman, &altitude_barometer);

    // altitude_sensor_fusion = kalman_filter_get_state(&altitude_and_velocity_kalman)[0][0];
    // vertical_velocity = kalman_filter_get_state(&altitude_and_velocity_kalman)[1][0];
    sensors11 = DWT->CYCCNT;

    // printf("Alt %f vel %f ", altitude_sensor_fusion, vertical_velocity);
    // printf("IMU %f %f %f\n", imu_orientation[0], imu_orientation[1], imu_orientation[2]);

    // printf("Lat %f, Lon %f\n", gps_latitude, gps_longitude);
    // printf("%f;%f;%f;\n", mangetometer_yaw_unfiltered, mangetometer_yaw_low_pass, magnetometer_yaw);

    // printf("YAW %f\n", mangetometer_yaw_low_pass);
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
    // This effectively turns the sensor around 180 degrees
    data[0] = -data[0];
    data[1] = -data[1];
}