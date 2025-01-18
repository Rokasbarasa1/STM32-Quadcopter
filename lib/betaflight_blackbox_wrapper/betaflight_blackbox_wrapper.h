#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void betaflight_blackbox_wrapper_get_header(
    uint32_t refresh_rate,
    float acro_roll_p, 
    float acro_roll_i, 
    float acro_roll_d, 
    float acro_pitch_p,
    float acro_pitch_i,
    float acro_pitch_d,
    float acro_yaw_p,
    float acro_yaw_i,
    float acro_yaw_d,
    float angle_mode_p,
    float angle_mode_i,
    float angle_mode_d,
    uint32_t yaw_lowpass,
    float acro_mode_roll_pitch_integral_windup,
    uint32_t gyro_lowpass_value,
    uint32_t accelerometer_lowpass_value,
    uint32_t pwm_frequency,
    uint16_t min_throttle, 
    uint16_t max_throttle, 
    uint16_t used_min_throttle, 
    uint16_t used_max_throttle, 
    uint8_t harmonics_filtered,
    uint16_t rpm_filter_q_value,
    uint16_t rpm_filter_min_hz,
    uint16_t d_term_low_pass_min,
    uint16_t d_term_low_pass_max,
    uint16_t d_term_low_pass_expo,
    uint16_t* string_length_return,
    uint8_t* buffer,
    uint16_t buffer_size,
    uint8_t mode
);

void betaflight_blackbox_get_encoded_data_string(
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
    float* motor_frequency,
    float* imu_orientation,
    float* angle_mode_targets,
    float altitude,
    uint16_t* string_length_return,
    uint8_t *buffer,
    uint8_t mode
);

char* betaflight_blackbox_get_end_of_log(uint16_t* string_length_return);

void betaflight_blackbox_get_encoded_gps_string(
    uint32_t time_raw,
    uint8_t sattalite_quantity,
    float latitude,
    float longitude,
    uint16_t* string_length_return,
    uint8_t *buffer
);