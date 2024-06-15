#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

char* betaflight_blackbox_wrapper_get_header(uint16_t min_throttle, uint16_t max_throttle, uint16_t* string_length_return);
char* betaflight_blackbox_get_encoded_data_string(
    uint32_t loop_iteration,
    uint32_t time,
    float* PID_proportion,
    float* PID_integral,
    float* PID_derivative,
    float* PID_feed_forward, 
    float* remote_control, // Roll, pitch, yaw, throttle
    float* set_points, // Targets for pid
    float* gyro_sums,
    float* accelerometer_values,
    float* motor_power,
    float* mag,
    float* gyro_post_sensor_fusion,
    float altitude,
    uint16_t* string_length_return
);
char* betaflight_blackbox_get_end_of_log(uint16_t* string_length_return);

char* betaflight_blackbox_get_encoded_gps_string(
    uint32_t time_raw,
    uint8_t number_of_satellites,
    float latitude,
    float longitude,
    float altitude,
    float speed,
    float ground_course,
    uint16_t* string_length_return
);