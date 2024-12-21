#include "./betaflight_blackbox_wrapper.h"

#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

uint16_t buffer_size(uint8_t* array){
    return strlen((char *)array);
}

void buffer_clear(uint8_t* array, uint16_t array_size){
    for(int i=0; i < array_size; i++){
        array[i] = 0;
    }
}

void buffer_append(uint8_t* array, uint16_t array_size, uint16_t *start_index, const char *string_format, ...){
    uint16_t current_length = *start_index;
    char* array_new = (char*)array;

    va_list args;
    va_start(args, string_format);
    // Ensure we don't write beyond the buffer
    int written = vsnprintf(&array_new[current_length], array_size - current_length, string_format, args);
    if (written > 0) {
        current_length += written < (array_size - current_length) ? written : (array_size - current_length - 1);
    }
    va_end(args);

    *start_index = current_length;
}

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
    uint16_t buffer_size
){
    uint16_t string_length = 0;

    uint32_t acro_roll_p_uint = floor(acro_roll_p * 100);
    uint32_t acro_roll_i_uint = floor(acro_roll_i * 100);
    uint32_t acro_roll_d_uint = floor(acro_roll_d * 10000); // Needs more precision

    uint32_t acro_pitch_p_uint = floor(acro_pitch_p * 100);
    uint32_t acro_pitch_i_uint = floor(acro_pitch_i * 100);
    uint32_t acro_pitch_d_uint = floor(acro_pitch_d * 10000);

    uint32_t acro_yaw_p_uint = floor(acro_yaw_p * 100);
    uint32_t acro_yaw_i_uint = floor(acro_yaw_i * 100);
    uint32_t acro_yaw_d_uint = floor(acro_yaw_d * 10000);

    uint32_t angle_mode_p_uint = floor(angle_mode_p * 100);
    uint32_t angle_mode_i_uint = floor(angle_mode_i * 100);
    uint32_t angle_mode_d_uint = floor(angle_mode_d * 10000);

    // float acro_mode_roll_pitch_integral_windup = 0;
    uint32_t acro_mode_roll_pitch_integral_windup_uint = floor(acro_mode_roll_pitch_integral_windup);

    uint16_t steps_for_1_G_acceleration = 16384;

    buffer_append(buffer, buffer_size, &string_length, "H Product:Blackbox flight data recorder by Nicholas Sherlock\n");
    buffer_append(buffer, buffer_size, &string_length, "H Data version:2\n");
    buffer_append(buffer, buffer_size, &string_length, "H I interval: 1\n");
    buffer_append(buffer, buffer_size, &string_length, "H P interval:1/1\n");

    // With mag and accelerometer
    // buffer_append(buffer, buffer_size, &string_length, "H Field I name:loopIteration,time,axisP[0],axisP[1],axisP[2],axisI[0],axisI[1],axisI[2],axisD[0],axisD[1],axisF[0],axisF[1],axisF[2],rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3],setpoint[0],setpoint[1],setpoint[2],setpoint[3],gyroADC[0],gyroADC[1],gyroADC[2],accSmooth[0],accSmooth[1],accSmooth[2],motor[0],motor[1],motor[2],motor[3],magADC[0],magADC[1],magADC[2],BaroAlt,debug[0],debug[1],debug[2],debug[3]\n");
    // buffer_append(buffer, buffer_size, &string_length, "H Field I signed:0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1\n");
    // buffer_append(buffer, buffer_size, &string_length, "H Field I predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n");
    // buffer_append(buffer, buffer_size, &string_length, "H Field I encoding:1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0\n");
    // buffer_append(buffer, buffer_size, &string_length, "H Field P predictor:6,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,1,1,1,1,1,1,1,1\n");
    // buffer_append(buffer, buffer_size, &string_length, "H Field P encoding:9,0,0,0,0,7,7,7,0,0,0,0,0,8,8,8,8,8,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n");

    // With accelerometer
    buffer_append(buffer, buffer_size, &string_length, "H Field I name:loopIteration,time,axisP[0],axisP[1],axisP[2],axisI[0],axisI[1],axisI[2],axisD[0],axisD[1],axisF[0],axisF[1],axisF[2],rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3],setpoint[0],setpoint[1],setpoint[2],setpoint[3],gyroADC[0],gyroADC[1],gyroADC[2],accSmooth[0],accSmooth[1],accSmooth[2],motor[0],motor[1],motor[2],motor[3],BaroAlt,debug[0],debug[1],debug[2],debug[3]\n");
    buffer_append(buffer, buffer_size, &string_length, "H Field I signed:0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1\n");
    buffer_append(buffer, buffer_size, &string_length, "H Field I predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n");
    buffer_append(buffer, buffer_size, &string_length, "H Field I encoding:1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0\n");
    buffer_append(buffer, buffer_size, &string_length, "H Field P predictor:6,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,1,1,1,1,1\n");
    buffer_append(buffer, buffer_size, &string_length, "H Field P encoding:9,0,0,0,0,7,7,7,0,0,0,0,0,8,8,8,8,8,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n");

    // Without mag and accelerometer
    // buffer_append(buffer, buffer_size, &string_length, "H Field I name:loopIteration,time,axisP[0],axisP[1],axisP[2],axisI[0],axisI[1],axisI[2],axisD[0],axisD[1],axisF[0],axisF[1],axisF[2],rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3],setpoint[0],setpoint[1],setpoint[2],setpoint[3],gyroADC[0],gyroADC[1],gyroADC[2],motor[0],motor[1],motor[2],motor[3],BaroAlt,debug[0],debug[1],debug[2],debug[3]\n");
    // buffer_append(buffer, buffer_size, &string_length, "H Field I signed:0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1\n");
    // buffer_append(buffer, buffer_size, &string_length, "H Field I predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n");
    // buffer_append(buffer, buffer_size, &string_length, "H Field I encoding:1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0\n");
    // buffer_append(buffer, buffer_size, &string_length, "H Field P predictor:6,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,1,1,1,1,1\n");
    // buffer_append(buffer, buffer_size, &string_length, "H Field P encoding:9,0,0,0,0,7,7,7,0,0,0,0,0,8,8,8,8,8,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0\n");

    // buffer_append(buffer, buffer_size, &string_length, "H Field G name:time,GPS_numSat,GPS_coord[0],GPS_coord[1],GPS_altitude,GPS_speed,GPS_ground_course\n");
    // buffer_append(buffer, buffer_size, &string_length, "H Field G signed:0,0,1,1,0,0,0\n");
    // buffer_append(buffer, buffer_size, &string_length, "H Field G predictor:0,0,0,0,0,0,0\n");
    // buffer_append(buffer, buffer_size, &string_length, "H Field G encoding:1,1,0,0,1,1,1\n");
    buffer_append(buffer, buffer_size, &string_length, "H Field G name:time,GPS_numSat,GPS_coord[0],GPS_coord[1]\n");
    buffer_append(buffer, buffer_size, &string_length, "H Field G signed:0,0,1,1\n");
    buffer_append(buffer, buffer_size, &string_length, "H Field G predictor:0,0,0,0\n");
    buffer_append(buffer, buffer_size, &string_length, "H Field G encoding:1,1,0,0\n");

    buffer_append(buffer, buffer_size, &string_length, "H Firmware type:Cleanflight\n"); // Lie
    buffer_append(buffer, buffer_size, &string_length, "H Firmware revision:Betaflight 4.3.1 (8d4f005) STM32F7X2\n"); // Lie
    buffer_append(buffer, buffer_size, &string_length, "H Firmware date:Jul 13 2022 03:36:10\n"); // Lie
    buffer_append(buffer, buffer_size, &string_length, "H Board information:Custom hardware by Rokas\n");
    buffer_append(buffer, buffer_size, &string_length, "H Log start datetime:0000-01-01T00:00:00.000+00:00\n");
    buffer_append(buffer, buffer_size, &string_length, "H Craft name:\n");
    buffer_append(buffer, buffer_size, &string_length, "H I interval:1\n"); // For every loop I is calculate
    buffer_append(buffer, buffer_size, &string_length, "H P interval:1\n"); // For every loop P is calculate
    buffer_append(buffer, buffer_size, &string_length, "H P ratio:1\n"); // ignore
    buffer_append(buffer, buffer_size, &string_length, "H minthrottle:%d\n", used_min_throttle);
    buffer_append(buffer, buffer_size, &string_length, "H maxthrottle:%d\n", used_max_throttle);
    // For some reason my logs have crazy values for the gyro when scale is set to 1.0
    // My actual scale is 131 = 1 deg/s
    // Settings i tried bellow and the results
    // 0.00763359           0x3bfa232d     With 0x3bfa232d the raw value of 2 is interpreted as 874745 deg/s.
    // 1                    0x3f800000     With 0x3f800000 the raw value of 2 is interpreted as 114591559 deg/s.
    // 131                  0x43030000     With 0x43030000 the raw value of 2 is interpreted as 15011494232 deg/s
    // So i just calculated it: 0.00763359 * (0.01526717557/874745) = 0.00000000013323124 (0x2f127d43 in float)
    // Now it shows up correctly despite the analyzer trying to fuck it up
    
    // buffer_append(buffer, buffer_size, &string_length, "H gyro_scale:0x3bfa232d\n"); 
    buffer_append(buffer, buffer_size, &string_length, "H gyro_scale:0x3f800000\n"); 
    buffer_append(buffer, buffer_size, &string_length, "H motorOutput:%d,%d\n", min_throttle, max_throttle);
    buffer_append(buffer, buffer_size, &string_length, "H acc_1G:%d\n", steps_for_1_G_acceleration);
    buffer_append(buffer, buffer_size, &string_length, "H looptime:%d\n", (1000000/refresh_rate)-1);  // microseconds to complete the control loop
    buffer_append(buffer, buffer_size, &string_length, "H gyro_sync_denom:1\n"); // How frequently is the gyro data processed. 1 = once per loop
    buffer_append(buffer, buffer_size, &string_length, "H pid_process_denom:1\n"); // How often is the pid processed? 1 = every time. 2 = every second time.
    buffer_append(buffer, buffer_size, &string_length, "H thr_mid:50\n"); // WHat throttle value is at the mid point? 50 = 0.5 or 50%
    buffer_append(buffer, buffer_size, &string_length, "H thr_expo:0\n"); // Npt used
    buffer_append(buffer, buffer_size, &string_length, "H tpa_rate:0\n"); // This reduces the pid master gain at full throttle. Dont have this functionality
    buffer_append(buffer, buffer_size, &string_length, "H tpa_breakpoint:%d\n", max_throttle); // WHen does the master pid gain start to decrease. What throttle. Max throttle as not used.
    buffer_append(buffer, buffer_size, &string_length, "H rc_rates:100,100,100\n"); // How does the radio control scale to the angular rates. It does not in our case so 100= 1.0 = maps directly.
    buffer_append(buffer, buffer_size, &string_length, "H rc_expo:0,0,0\n"); // How the quadcopter will respond to the values at the stick center point. High value makes it less responsive. Not used
    buffer_append(buffer, buffer_size, &string_length, "H rates:0,0,0\n"); // How much are angle rates boosted at high throttles
    buffer_append(buffer, buffer_size, &string_length, "H rate_limits:%d,%d,%d\n", max_throttle, max_throttle, max_throttle); // At what motor value does the boost come in. Max throttle as not used
    buffer_append(buffer, buffer_size, &string_length, "H rollPID:%lu,%lu,%lu\n", acro_roll_p_uint, acro_roll_i_uint, acro_roll_d_uint); // Acro mode roll PID gains. 0.003 = 3
    buffer_append(buffer, buffer_size, &string_length, "H pitchPID:%lu,%lu,%lu\n", acro_pitch_p_uint, acro_pitch_i_uint, acro_pitch_d_uint); // Acro mode pitchPID gains
    buffer_append(buffer, buffer_size, &string_length, "H yawPID:%lu,%lu,%lu\n", acro_yaw_p_uint, acro_yaw_i_uint, acro_yaw_d_uint); // Acro mode yaw PID gains
    buffer_append(buffer, buffer_size, &string_length, "H levelPID:%lu,%lu,%lu\n", angle_mode_p_uint, angle_mode_i_uint, angle_mode_d_uint); // ANgle mode PID
    buffer_append(buffer, buffer_size, &string_length, "H magPID:0\n"); // Magnetometer yaw PID, not used now
    buffer_append(buffer, buffer_size, &string_length, "H d_min:40,40,0\n"); // Dont use it
    buffer_append(buffer, buffer_size, &string_length, "H d_max_gain:40\n"); // Dont use it
    buffer_append(buffer, buffer_size, &string_length, "H d_max_advance:0\n"); // Dont use it
    buffer_append(buffer, buffer_size, &string_length, "H dterm_lpf1_type:2\n"); // 2 = BIQUAD
    buffer_append(buffer, buffer_size, &string_length, "H dterm_lpf1_static_hz:0\n"); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H dterm_lpf1_dyn_hz:%d,%d\n",d_term_low_pass_min, d_term_low_pass_max); // The d term filter min an max
    buffer_append(buffer, buffer_size, &string_length, "H dterm_lpf1_dyn_expo:%d\n", d_term_low_pass_expo); // Not yet implemented
    buffer_append(buffer, buffer_size, &string_length, "H dterm_lpf2_type:0\n"); // Not yet implemented
    buffer_append(buffer, buffer_size, &string_length, "H dterm_lpf2_static_hz:0\n"); // Not yet implemented
    buffer_append(buffer, buffer_size, &string_length, "H yaw_lowpass_hz:%d\n", yaw_lowpass); // What is the low pass cutoff frequency for the yaw in acro mode
    buffer_append(buffer, buffer_size, &string_length, "H dterm_notch_hz:0\n"); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H dterm_notch_cutoff:0\n"); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H iterm_windup:%lu\n", acro_mode_roll_pitch_integral_windup_uint);
    buffer_append(buffer, buffer_size, &string_length, "H iterm_relax:0\n");
    buffer_append(buffer, buffer_size, &string_length, "H iterm_relax_type:0\n");
    buffer_append(buffer, buffer_size, &string_length, "H iterm_relax_cutoff:100\n");
    buffer_append(buffer, buffer_size, &string_length, "H pid_at_min_throttle:1\n"); // When the throttle is all the way down the PID loop will still work. Will never change
    buffer_append(buffer, buffer_size, &string_length, "H anti_gravity_mode:0\n"); // WIll not change this
    buffer_append(buffer, buffer_size, &string_length, "H anti_gravity_threshold:250\n");
    buffer_append(buffer, buffer_size, &string_length, "H anti_gravity_gain:6000\n");
    buffer_append(buffer, buffer_size, &string_length, "H abs_control_gain:0\n");
    buffer_append(buffer, buffer_size, &string_length, "H use_integrated_yaw:0\n");
    buffer_append(buffer, buffer_size, &string_length, "H ff_weight:0,0,0\n"); // Currently not implemented 
    buffer_append(buffer, buffer_size, &string_length, "H feedforward_transition:0\n"); // Currently not implemented 
    buffer_append(buffer, buffer_size, &string_length, "H feedforward_averaging:1\n"); // Currently not implemented 
    buffer_append(buffer, buffer_size, &string_length, "H feedforward_smooth_factor:65\n"); // Currently not implemented 
    buffer_append(buffer, buffer_size, &string_length, "H feedforward_jitter_factor:7\n"); // Currently not implemented 
    buffer_append(buffer, buffer_size, &string_length, "H feedforward_boost:15\n"); // Currently not implemented 
    buffer_append(buffer, buffer_size, &string_length, "H feedforward_max_rate_limit:90\n"); // Currently not implemented 
    buffer_append(buffer, buffer_size, &string_length, "H acc_limit_yaw:0\n");
    buffer_append(buffer, buffer_size, &string_length, "H acc_limit:0\n");
    buffer_append(buffer, buffer_size, &string_length, "H pidsum_limit:500\n"); // Not used and the value set here is more than enough
    buffer_append(buffer, buffer_size, &string_length, "H pidsum_limit_yaw:400\n"); // Not used and the value set here is more than enough
    buffer_append(buffer, buffer_size, &string_length, "H deadband:0\n"); // This is set by the remote. So not used
    buffer_append(buffer, buffer_size, &string_length, "H yaw_deadband:0\n"); // This is set by the remote. So not used
    buffer_append(buffer, buffer_size, &string_length, "H gyro_hardware_lpf:%d\n", gyro_lowpass_value); // A hardware gyro lowpass is used 
    buffer_append(buffer, buffer_size, &string_length, "H gyro_lpf1_type:0\n"); // LPF1
    buffer_append(buffer, buffer_size, &string_length, "H gyro_lpf1_static_hz:0\n");
    buffer_append(buffer, buffer_size, &string_length, "H gyro_lpf1_dyn_hz:0,750\n");
    buffer_append(buffer, buffer_size, &string_length, "H gyro_lpf2_type:0\n"); // LPF1
    buffer_append(buffer, buffer_size, &string_length, "H gyro_lpf2_static_hz:750\n");
    buffer_append(buffer, buffer_size, &string_length, "H gyro_notch_hz:0,0\n"); // not used
    buffer_append(buffer, buffer_size, &string_length, "H gyro_notch_cutoff:0,0\n");// Not used
    buffer_append(buffer, buffer_size, &string_length, "H gyro_to_use:0\n");// Default gyro, no other one on this drone
    buffer_append(buffer, buffer_size, &string_length, "H dyn_notch_max_hz:0\n"); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H dyn_notch_count:0\n"); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H dyn_notch_q:0\n"); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H dyn_notch_min_hz:0\n"); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H dshot_bidir:1\n"); // It does use bdshot600
    buffer_append(buffer, buffer_size, &string_length, "H motor_poles:12\n"); // My eco II use 12 poles. Not sure why it needs to know this
    buffer_append(buffer, buffer_size, &string_length, "H rpm_filter_harmonics:%d\n", harmonics_filtered); // 
    buffer_append(buffer, buffer_size, &string_length, "H rpm_filter_q:%d\n", rpm_filter_q_value); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H rpm_filter_min_hz:%d\n", rpm_filter_min_hz); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H rpm_filter_fade_range_hz:0\n"); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H rpm_filter_lpf_hz:0\n"); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H acc_lpf_hz:%d\n", accelerometer_lowpass_value); // This setting is for showing software filtering, but i still want to know the filtering even if it is hardware one
    buffer_append(buffer, buffer_size, &string_length, "H acc_hardware:1\n");
    buffer_append(buffer, buffer_size, &string_length, "H baro_hardware:1\n");
    buffer_append(buffer, buffer_size, &string_length, "H mag_hardware:1\n");
    buffer_append(buffer, buffer_size, &string_length, "H gyro_cal_on_first_arm:1\n"); // Gyro is indeed calibrated durring arming
    buffer_append(buffer, buffer_size, &string_length, "H airmode_activate_throttle:0\n"); // Not using this
    buffer_append(buffer, buffer_size, &string_length, "H serialrx_provider:9\n"); // Does not matter, leave the same
    buffer_append(buffer, buffer_size, &string_length, "H use_unsynced_pwm:0\n"); // pwm is updated in the same loop as pid calculation
    buffer_append(buffer, buffer_size, &string_length, "H motor_pwm_protocol:6\n"); // 6 = DSHOT600 
    buffer_append(buffer, buffer_size, &string_length, "H motor_pwm_rate:%d\n", pwm_frequency);
    buffer_append(buffer, buffer_size, &string_length, "H dshot_idle_value:0\n");
    buffer_append(buffer, buffer_size, &string_length, "H debug_mode:6\n"); // 6 = Gyro scaled.
    buffer_append(buffer, buffer_size, &string_length, "H features:272958472\n"); // Bit mask for betaflight to read, no idea what to set this
    buffer_append(buffer, buffer_size, &string_length, "H rc_smoothing:1");
    buffer_append(buffer, buffer_size, &string_length, "H rc_smoothing_auto_factor:0\n");
    buffer_append(buffer, buffer_size, &string_length, "H rc_smoothing_auto_factor_throttle:0\n");
    buffer_append(buffer, buffer_size, &string_length, "H rc_smoothing_feedforward_cutoff:0\n");
    buffer_append(buffer, buffer_size, &string_length, "H rc_smoothing_setpoint_cutoff:0\n");
    buffer_append(buffer, buffer_size, &string_length, "H rc_smoothing_throttle_cutoff:0\n");
    buffer_append(buffer, buffer_size, &string_length, "H rc_smoothing_rx_average:0\n");
    buffer_append(buffer, buffer_size, &string_length, "H rates_type:0\n"); // Not needed
    buffer_append(buffer, buffer_size, &string_length, "H fields_disabled_mask:0\n"); // Not needed
    buffer_append(buffer, buffer_size, &string_length, "H vbat_sag_compensation:0\n");  // Not needed
    buffer_append(buffer, buffer_size, &string_length, "H dyn_idle_min_rpm:0\n");  // Not needed
    buffer_append(buffer, buffer_size, &string_length, "H dyn_idle_p_gain:0\n");  // Not needed
    buffer_append(buffer, buffer_size, &string_length, "H dyn_idle_i_gain:0\n");  // Not needed
    buffer_append(buffer, buffer_size, &string_length, "H dyn_idle_d_gain:0\n");  // Not needed
    buffer_append(buffer, buffer_size, &string_length, "H dyn_idle_max_increase:0\n");  // Not needed
    buffer_append(buffer, buffer_size, &string_length, "H simplified_pids_mode:0\n"); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H motor_output_limit:100\n"); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H throttle_limit_type:0\n"); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H throttle_limit_percent:100\n"); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H throttle_boost:0\n"); // Not used
    buffer_append(buffer, buffer_size, &string_length, "H throttle_boost_cutoff:0\n"); // Not used

    *string_length_return = string_length;
}

/**
 * Cast the in-memory representation of the given float directly to an int.
 *
 * This is useful for printing the hex representation of a float number (which is considerably cheaper
 * than a full decimal float formatter, in both code size and output length).
 */
uint32_t cast_float_bytes_to_int(float f)
{
    union floatConvert_t {
        float f;
        uint32_t u;
    } floatConvert;

    floatConvert.f = f;

    return floatConvert.u;
}

/**
 * ZigZag encoding maps all values of a signed integer into those of an unsigned integer in such
 * a way that numbers of small absolute value correspond to small integers in the result.
 *
 * (Compared to just casting a signed to an unsigned which creates huge resulting numbers for
 * small negative integers).
 */
uint32_t zigzag_encode(int32_t value)
{
    return (uint32_t)((value << 1) ^ (value >> 31));
}

/**
 * Write an unsigned integer to the blackbox serial port using variable byte encoding.
 */
void blackbox_write_unsigned_VB(uint32_t value, uint8_t* byte_array, uint16_t* byte_array_index){
    // Lets assume that there is always enough space in the array
    
    // While this isn't the final byte (we can only write 7 bits at a time)
    while (value > 127) {
        byte_array[(*byte_array_index)++] = (uint8_t) (value | 0x80); // Set the high bit
        value >>= 7;
    }
    byte_array[(*byte_array_index)++] = value;
}

/**
 * Write a signed integer to the blackbox serial port using ZigZig and variable byte encoding.
 */
void blackbox_write_signed_VB(int32_t value, uint8_t* byte_array, uint16_t* byte_array_index){
    //ZigZag encode to make the value always positive
    blackbox_write_unsigned_VB(zigzag_encode(value), byte_array, byte_array_index);
}

void blackbox_write_signed_VB_array(int32_t *array, int count, uint8_t* byte_array, uint16_t* byte_array_index){
    for (int i = 0; i < count; i++) {
        blackbox_write_signed_VB(array[i], byte_array, byte_array_index);
    }
}

/** Write unsigned integer **/
void blackbox_write_U32(int32_t value, uint8_t* byte_array, uint16_t* byte_array_index){

    byte_array[(*byte_array_index)++] = value & 0xFF;
    byte_array[(*byte_array_index)++] = (value >> 8) & 0xFF;
    byte_array[(*byte_array_index)++] = (value >> 16) & 0xFF;
    byte_array[(*byte_array_index)++] = (value >> 24) & 0xFF;
}

/** Write float value in the integer form **/
void blackbox_write_float(float value, uint8_t* byte_array, uint16_t* byte_array_index){
    blackbox_write_U32(cast_float_bytes_to_int(value), byte_array, byte_array_index);
}

void blackbox_write_float_array(float *array, int count, uint8_t* byte_array, uint16_t* byte_array_index){
    for (int i = 0; i < count; i++) {
        blackbox_write_float(array[i], byte_array, byte_array_index);
    }
}

void blackbox_write_signed_16VB_array(int16_t *array, int count, uint8_t* byte_array, uint16_t* byte_array_index){
    for (int i = 0; i < count; i++) {
        blackbox_write_signed_VB(array[i], byte_array, byte_array_index);
    }
}

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
    float altitude,
    uint16_t* string_length_return,
    uint8_t *buffer
){
    uint16_t string_index = 0;

    
    uint16_t scaling_factor = 10;
    uint16_t gyro_scaling_factor = 1; // This is to convert it to raw degrees
    uint16_t accelerometer_scaling_factor = 16384;

    int32_t PID_proportion_int[3] = {(int32_t)(PID_proportion[0]*scaling_factor), (int32_t)(PID_proportion[1]*scaling_factor), (int32_t)(PID_proportion[2]*scaling_factor)};
    int32_t PID_integral_int[3] = {(int32_t)(PID_integral[0]*scaling_factor), (int32_t)(PID_integral[1]*scaling_factor), (int32_t)(PID_integral[2]*scaling_factor)};
    int32_t PID_derivative_int[3] = {(int32_t)(PID_derivative[0]*scaling_factor), (int32_t)(PID_derivative[1]*scaling_factor), (int32_t)(PID_derivative[2]*scaling_factor)};
    int32_t PID_feed_forward_int[3] = {(int32_t)(PID_feed_forward[0]*scaling_factor), (int32_t)(PID_feed_forward[1]*scaling_factor), (int32_t)(PID_feed_forward[2]*scaling_factor)};

    int16_t remote_control_int[4] = {(int16_t)(remote_control[0]*scaling_factor), (int16_t)(remote_control[1]*scaling_factor), (int16_t)(remote_control[2]*scaling_factor), (int16_t)(remote_control[3]*scaling_factor)};
    int16_t set_points_int[4] = {(int16_t)(set_points[0]), (int16_t)(set_points[1]), (int16_t)(set_points[2]), (int16_t)(set_points[3])}; // For some reason the setpoints dont have to be scaled for degrees, they just straight up convert to degrees and the setpoint3 does as well
    int16_t gyro_sums_int[3] = {(int16_t)(gyro_sums[0]*gyro_scaling_factor), (int16_t)(gyro_sums[1]*gyro_scaling_factor), (int16_t)(gyro_sums[2]*gyro_scaling_factor)};
    int16_t accelerometer_values_int[3] = {(int16_t)(accelerometer_values[0]*accelerometer_scaling_factor), (int16_t)(accelerometer_values[1]*accelerometer_scaling_factor), (int16_t)(accelerometer_values[2]*accelerometer_scaling_factor)};
    uint16_t motor_power_int[4] = {
        (uint16_t)motor_power[2], 
        (uint16_t)motor_power[3], 
        (uint16_t)motor_power[0],
        (uint16_t)motor_power[1],
    };

    int32_t mag_int[3] = {(int32_t)(mag[0]*scaling_factor), (int32_t)(mag[1]*scaling_factor), (int32_t)(mag[2]*scaling_factor)};

    int32_t gyro_post_sensor_fusion_int[4] = {(int32_t)(motor_frequency[0]*gyro_scaling_factor), (int32_t)(motor_frequency[1]*gyro_scaling_factor), (int32_t)(motor_frequency[2]*gyro_scaling_factor), (int32_t)(motor_frequency[3]*gyro_scaling_factor)}; // added the motor frequency average to debug index 3
    int32_t altitude_int = (int32_t)(altitude*scaling_factor); // 10 float value is 1.0 meter after it arrives to the logger.
    
    // float scaling_factor = 10;
    // float gyro_scaling_factor = 131.0; // This is to convert it to raw degrees
    // float accelerometer_scaling_factor = 16384.0;

    // int32_t PID_proportion_int[3] = {lrintf(PID_proportion[0]*scaling_factor), lrintf(PID_proportion[1]*scaling_factor), lrintf(PID_proportion[2]*scaling_factor)};
    // int32_t PID_integral_int[3] = {lrintf(PID_integral[0]*scaling_factor), lrintf(PID_integral[1]*scaling_factor), lrintf(PID_integral[2]*scaling_factor)};
    // int32_t PID_derivative_int[3] = {lrintf(PID_derivative[0]*scaling_factor), lrintf(PID_derivative[1]*scaling_factor), lrintf(PID_derivative[2]*scaling_factor)};
    // int32_t PID_feed_forward_int[3] = {lrintf(PID_feed_forward[0]*scaling_factor), lrintf(PID_feed_forward[1]*scaling_factor), lrintf(PID_feed_forward[2]*scaling_factor)};

    // int16_t remote_control_int[4] = {lrintf(remote_control[0]*scaling_factor), lrintf(remote_control[1]*scaling_factor), lrintf(remote_control[2]*scaling_factor), lrintf(remote_control[3]*scaling_factor)};
    // int16_t set_points_int[4] = {lrintf(set_points[0]*gyro_scaling_factor), lrintf(set_points[1]*gyro_scaling_factor), lrintf(set_points[2]*gyro_scaling_factor), lrintf(set_points[3]*gyro_scaling_factor)};
    // int16_t gyro_sums_int[3] = {lrintf(gyro_sums[0]*gyro_scaling_factor), lrintf(gyro_sums[1]*gyro_scaling_factor), lrintf(gyro_sums[2]*gyro_scaling_factor)};
    // int16_t accelerometer_values_int[3] = {lrintf(accelerometer_values[0]*accelerometer_scaling_factor), lrintf(accelerometer_values[1]*accelerometer_scaling_factor), lrintf(accelerometer_values[2]*accelerometer_scaling_factor)};
    // uint16_t motor_power_int[4] = {
    //     lrintf(motor_power[2]), 
    //     lrintf(motor_power[3]), 
    //     lrintf(motor_power[0]),
    //     lrintf(motor_power[1]),
    // };

    // int32_t mag_int[3] = {lrintf(mag[0]*scaling_factor), lrintf(mag[1]*scaling_factor), lrintf(mag[2]*scaling_factor)};

    // int32_t gyro_post_sensor_fusion_int[4] = {lrintf(gyro_post_sensor_fusion[0]*gyro_scaling_factor), lrintf(gyro_post_sensor_fusion[1]*gyro_scaling_factor), lrintf(gyro_post_sensor_fusion[2]*gyro_scaling_factor), 0};
    // int32_t altitude_int = lrintf(altitude*scaling_factor); // 10 float value is 1.0 meter after it arrives to the logger.





    // The big writes are not that meaningful here as in the header one, assume everything is ugly bytes here
    buffer[string_index++] = 'I';
    blackbox_write_unsigned_VB(loop_iteration, buffer, &string_index); // loopIteration 0 1 GOOD
    blackbox_write_unsigned_VB(time, buffer, &string_index); // time 0 1 GOOD
    blackbox_write_signed_VB_array(PID_proportion_int, 3, buffer, &string_index); // axisP[0],axisP[1],axisP[2] 1,1,1 0,0,0 GOOD
    blackbox_write_signed_VB_array(PID_integral_int, 3, buffer, &string_index); // axisI[0],axisI[1],axisI[2] 1,1,1 0,0,0 GOOD
    blackbox_write_signed_VB_array(PID_derivative_int, 2, buffer, &string_index); //axisD[0],axisD[1] 1,1 0,0 GOOD
    blackbox_write_signed_VB_array(PID_feed_forward_int, 3, buffer, &string_index); // axisF[0],axisF[1],axisF[2] 1,1,1 0,0,0 GOOD
    // yaw is inverted when you go left it goes right
    blackbox_write_signed_16VB_array(remote_control_int, 3, buffer, &string_index); // rcCommand[0],rcCommand[1],rcCommand[2] 1,1,1 0,0,0 GOOD
    blackbox_write_unsigned_VB(remote_control_int[3] ,buffer, &string_index); // rcCommand[3] 0 1 GOOD
    blackbox_write_signed_16VB_array(set_points_int, 4, buffer, &string_index); // setpoint[0],setpoint[1],setpoint[2],setpoint[3] 1,1,1,1 0,0,0,0 NOT
    blackbox_write_signed_16VB_array(gyro_sums_int, 3, buffer, &string_index); // gyroADC[0],gyroADC[1],gyroADC[2] 1,1,1 0,0,0 GOOD
    blackbox_write_signed_16VB_array(accelerometer_values_int, 3, buffer, &string_index); // accG[0],accG[1],accG[2] 1,1,1 0,0,0 GOOD
    blackbox_write_unsigned_VB(motor_power_int[0], buffer, &string_index); // motor[0] 0 1 GOOD
    blackbox_write_unsigned_VB(motor_power_int[1], buffer, &string_index); // motor[1] 0 1 GOOD
    blackbox_write_unsigned_VB(motor_power_int[2], buffer, &string_index); // motor[2] 0 1 GOOD
    blackbox_write_unsigned_VB(motor_power_int[3], buffer, &string_index); // motor[3] 0 1 GOOD

    // blackbox_write_signed_VB_array(mag_int, 3, buffer, &string_index); // magADC[0],magADC[1],magADC[2] 1,1,1 0,0,0
    blackbox_write_signed_VB(altitude_int, buffer, &string_index); // BaroAlt 1 0
    blackbox_write_signed_VB_array(gyro_post_sensor_fusion_int, 4, buffer, &string_index); // debug[0],debug[1],debug[2],debug[3] 1,1,1,1 0,0,0,0

    // printf("loopIteration=%ld\n", loop_iteration);
    // printf("time=%ld\n", time);
    // printf("axisP[0]=%ld ,axisP[1]=%ld ,axisP[2]=%ld \n", PID_proportion_int[0], PID_proportion_int[1], PID_proportion_int[2]);
    // printf("axisI[0]=%ld ,axisI[1]=%ld ,axisI[2]=%ld \n", PID_integral_int[0], PID_integral_int[1], PID_integral_int[2]);
    // printf("axisD[0]=%ld ,axisD[1]=%ld \n", PID_derivative_int[0], PID_derivative_int[1]);
    // printf("axisF[0]=%ld ,axisF[1]=%ld ,axisF[2]=%ld \n", PID_feed_forward_int[0], PID_feed_forward_int[1], PID_feed_forward_int[2]);
    // printf("rcCommand[0]=%d ,rcCommand[1]=%d ,rcCommand[2]=%d ,rcCommand[3]=%d\n", remote_control_int[0], remote_control_int[1], remote_control_int[2], remote_control_int[3]);
    // printf("setpoint[0]=%d ,setpoint[1]=%d ,setpoint[2]=%d ,setpoint[3]=%d\n", set_points_int[0], set_points_int[1], set_points_int[2], set_points_int[3]);
    // printf("gyroADC[0]=%d ,gyroADC[1]=%d ,gyroADC[2]=%d\n", gyro_sums_int[0], gyro_sums_int[1], gyro_sums_int[2]);
    // printf("accG[0]=%d ,accG[1]=%d ,accG[2]=%d\n", accelerometer_values_int[0], accelerometer_values_int[1], accelerometer_values_int[2]);
    // printf("motor[0]=%d ,motor[1]=%d ,motor[2]=%d ,motor[3]=%d\n", motor_power_int[0], motor_power_int[1], motor_power_int[2], motor_power_int[3]);
    // printf("magADC[0]=%d ,magADC[1]=%d ,magADC[2]=%d\n", mag_int[0], mag_int[1], mag_int[2]);
    // printf("BaroAlt=%ld\n", altitude_int);
    // printf("debug[0]=%d ,debug[1]=%d ,debug[2]=%d ,debug[3]=%d\n", gyro_post_sensor_fusion_int[0], gyro_post_sensor_fusion_int[1], gyro_post_sensor_fusion_int[2], gyro_post_sensor_fusion_int[3]);
    // printf("Index after: %d\n", string_index);
    // printf("\n");

    // printf("%d Create ", string_index);
    // for (uint16_t i = 0; i < string_index; i++){
    //     printf("%02X ", buffer[i]);
    // }
    // printf("\n");

    *string_length_return += string_index;
}

char* betaflight_blackbox_get_end_of_log(uint16_t* string_length_return){
    uint16_t string_length_total = 11;
    char* new_string = malloc(string_length_total+1);
    buffer_append(new_string, string_length_total, 0, "End of log\n");
    *string_length_return = string_length_total;

    return new_string;
}

void betaflight_blackbox_get_encoded_gps_string(
    uint32_t time_raw,
    uint8_t fix_quality,
    float latitude,
    float longitude,
    uint16_t* string_length_return,
    uint8_t *buffer
){
    uint16_t string_index = 0;

    float scaling_factor = 10;
    // time
    // number_of_satellites
    int32_t latitude_int = (int32_t)latitude*10.0;
    int32_t longitude_int = (int32_t)longitude*10.0;

    buffer[string_index++] = 'G';

    blackbox_write_unsigned_VB(time_raw, buffer, &string_index); // time 0 1 GOOD
    blackbox_write_unsigned_VB(fix_quality, buffer, &string_index); // GPS_numSat 0 1 GOOD
    blackbox_write_signed_VB(latitude_int, buffer, &string_index); // GPS_coord[0] 1 0
    blackbox_write_signed_VB(longitude_int, buffer, &string_index); // GPS_coord[1] 1 0
    // blackbox_write_unsigned_VB(altitude_int, buffer, &string_index); // GPS_altitude 0 1 GOOD
    // blackbox_write_unsigned_VB(speed_int, buffer, &string_index); // GPS_speed 0 1 GOOD
    // blackbox_write_unsigned_VB(ground_course_int, buffer, &string_index); // GPS_ground_course 0 1 GOOD

    // printf("time=%ld\n", time_raw);
    // printf("number_of_satellites=%ld\n", time_raw);
    // printf("latitude_int=%ld\n", latitude_int);
    // printf("longitude_int=%ld\n", longitude_int);
    // printf("altitude_int=%ld\n", altitude_int);
    // printf("speed_int=%ld\n", speed_int);
    // printf("ground_course_int=%ld\n", ground_course_int);

    *string_length_return += string_index;
}