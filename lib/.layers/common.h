#include "stm32f4xx_hal.h"
#include "system.h"

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern DMA_HandleTypeDef hdma_spi3_tx;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

#include "../lib/printf/retarget.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include <inttypes.h>

// Drivers 
#include "../lib/utils/i2c_scanner/i2c_scanner.h"

#include "../lib/mpu6050/mpu6050.h"
#include "../lib/qmc5883l/qmc5883l.h"
#include "../lib/hmc5883l/hmc5883l.h"
#include "../lib/bmm350/bmm350.h"
#include "../lib/ist8310/ist8310.h"

#include "../lib/mmc5603/mmc5603.h"
#include "../lib/ms5611/ms5611.h"
#include "../lib/bn357/bn357.h"
#include "../lib/nrf24l01/nrf24l01.h"
#include "../lib/sd_card/sd_card_spi.h"
#include "../lib/betaflight_blackbox_wrapper/betaflight_blackbox_wrapper.h"

// Other imports
#include "../lib/utils/ned_coordinates/ned_coordinates.h"
#include "../lib/pid/pid.h"
#include "../lib/filtering/filtering.h"
#include "../lib/filtering/iir_filter.h"
#include "../lib/filtering/moving_average.h"
#include "../lib/filtering/outlier_detection.h"

#include "../lib/kalman_filter/kalman_filter.h"
#include "../lib/utils/matrix_operations/matrix_operations.h"
#include "../lib/bdshot600_dma/bdshot600_dma.h"
#include "../utils/math_constants.h"
#include "../utils/reset_i2c/reset_i2c.h"

extern uint32_t radio;
extern uint32_t radio2;
extern uint32_t radio_end;

extern uint32_t sensors;
extern uint32_t sensors2;
extern uint32_t sensors3;
extern uint32_t sensors4;
extern uint32_t sensors5;
extern uint32_t sensors6;
extern uint32_t sensors7;
extern uint32_t sensors8;
extern uint32_t sensors9;
extern uint32_t sensors10;
extern uint32_t sensors11;
extern uint32_t sensors_end;

extern uint32_t motor;
extern uint32_t motor2;
extern uint32_t motor3;
extern uint32_t motor4;
extern uint32_t motor5;
extern uint32_t motor6;
extern uint32_t motor7;
extern uint32_t motor_end;

extern uint32_t logging;
extern uint32_t logging2;
extern uint32_t logging3;
extern uint32_t logging_end;


extern uint32_t interrupt;
extern uint32_t interrupt_end;

// Nyquist: sampling frequency must be at least double that of the highest frequency of interest
// The MPU6050 has a lowpass set to 260Hz we need to be able to manipulate all the frequencies until that. So 260 * 2 = 520
#define REFRESH_RATE_HZ 520

extern const uint16_t precalculated_timing_miliseconds;

extern uint64_t absolute_microseconds_since_start;
extern uint64_t loop_start_timestamp_microseconds;
extern uint64_t delta_time_without_waiting_from_previous_loop_microseconds;
extern uint64_t delta_time_total_loop_time_previous_loop_microseconds;


extern uint16_t loop_start_microseconds;
extern uint16_t loop_start_miliseconds;

extern uint32_t startup_time_microseconds;
extern uint32_t delta_time;
extern uint8_t time_since_startup_hours;
extern uint8_t time_since_startup_minutes;
extern uint8_t time_since_startup_seconds;
extern uint16_t time_since_startup_ms;
extern uint16_t time_since_startup_microseconds;

extern uint8_t entered_loop;

extern const uint32_t dshot_refresh_rate;

#define max_dshot600_throttle_value 2047
#define min_dshot600_throttle_value 48

#define max_dshot600_command_value 47
#define min_dshot600_command_value 1

#define dshot600_neutral_value 0
#define actual_min_dshot600_throttle_value 91 // Lowest value that lets the motors spin freely and at low rpm

extern const uint16_t actual_max_dshot600_throttle_value; 

extern float motor_power[];
extern float motor_frequency[];
extern float motor_rpm[];

extern float throttle_value; 
extern uint8_t motor_rotor_poles;

extern uint8_t motor_BL;
extern uint8_t motor_BR;
extern uint8_t motor_FR;
extern uint8_t motor_FL;

extern uint16_t throttle_value_FR;
extern uint16_t throttle_value_FL;
extern uint16_t throttle_value_BL;
extern uint16_t throttle_value_BR;

extern uint8_t manual_bdshot;


extern float magnetometer_mmc5603_hard_iron_correction[3];

extern float magnetometer_mmc5603_soft_iron_correction[][3];
extern float magnetometer_mmc5603_rotation_correction[3][3];



extern float magnetometer_hard_iron_correction[3];

extern float magnetometer_soft_iron_correction[][3];

extern float magnetometer_rotation_correction[3][3];



extern float magnetometer_ist8310_hard_iron_correction[3];

extern float magnetometer_ist8310_soft_iron_correction[][3];


extern float accelerometer_correction[];

extern float accelerometer_scale_factor_correction[][3];

extern float gyro_correction[];

extern float base_accelerometer_roll_offset;
extern float base_accelerometer_pitch_offset;

extern float accelerometer_roll_offset;
extern float accelerometer_pitch_offset;

extern float yaw_offset1;
extern float yaw_offset2;

extern float yaw_declination;

extern float accelerometer_roll;
extern float accelerometer_pitch;

extern float gyro_yaw;
extern float magnetometer_yaw_unrotated_no_tilt;
extern float magnetometer_yaw_unrotated_tilt;
extern float magnetometer_yaw_no_tilt;
extern float magnetometer_yaw;
extern float magnetometer_yaw_unrotated;
extern float magnetometer_yaw_secondary;


extern float magnetometer_yaw_90;
extern float magnetometer_yaw_180;
extern float magnetometer_yaw_270;
extern float magnetometer_yaw_secondary_unrotated;

extern float magnetometer_yaw_unfiltered_no_tilt;

extern float magnetometer_ist8310_yaw_unrotated_no_tilt;
extern float magnetometer_ist8310_yaw_unrotated_tilt;
extern float magnetometer_ist8310_yaw_no_tilt;
extern float magnetometer_ist8310_yaw;
extern float magnetometer_ist8310_yaw_unfiltered_no_tilt;

extern float magnetometer_data_unfiltered[];
extern float magnetometer_data_ist8310_unfiltered[];


extern uint8_t tx_address[];
extern char rx_data[];
extern char rx_type[];

extern struct pid acro_roll_pid;
extern struct pid acro_pitch_pid;
extern struct pid acro_yaw_pid;

extern struct pid angle_pitch_pid;
extern struct pid angle_roll_pid;

extern struct pid altitude_hold_pid;

extern struct pid gps_longitude_pid;
extern struct pid gps_latitude_pid;

extern float error_acro_roll;
extern float error_acro_pitch;
extern float error_acro_yaw;

extern float error_angle_pitch;
extern float error_angle_roll;

extern float error_throttle;
extern float error_vertical_velocity;
extern float error_altitude_barometer;

extern float error_latitude;
extern float error_longitude;
extern float last_error_roll;
extern float last_error_pitch;


// Notes for PID
// 1) It is important that the natural state without power of the drone is not stable, otherwise 
// some offset of center off mass or pid desired point needs to introduced
// 2) Remember that the drone when in the air has motors spinning at idle power, just enough 
// to float in the air. If you are testing with drone constrained add a base motor speed to account for this.


// Actual PID adjustment for pitch
#define BASE_ANGLE_roll_pitch_MASTER_GAIN 0.70
#define BASE_ANGLE_roll_pitch_GAIN_P 14.0
#define BASE_ANGLE_roll_pitch_GAIN_I 150.0
#define BASE_ANGLE_roll_pitch_GAIN_D 0.0 // Very bad results when used

#define BASE_ACRO_roll_pitch_MASTER_GAIN 0.65
#define BASE_ACRO_roll_pitch_GAIN_P 0.58
#define BASE_ACRO_roll_pitch_GAIN_I 0.0
#define BASE_ACRO_roll_pitch_GAIN_D 0.01

#define BASE_ALTITUDE_HOLD_MASTER_GAIN 1.0
#define BASE_ALTITUDE_HOLD_GAIN_P 0.1 // 1.15
#define BASE_ALTITUDE_HOLD_GAIN_I 0.0 // 0.0010
#define BASE_ALTITUDE_HOLD_GAIN_D 0.0

#define BASE_GPS_HOLD_MASTER_GAIN 1.0
#define BASE_GPS_HOLD_GAIN_P 0.3
#define BASE_GPS_HOLD_GAIN_I 0.0
#define BASE_GPS_HOLD_GAIN_D 0.0


extern float angle_roll_pitch_master_gain;
extern float angle_roll_pitch_gain_p;
extern float angle_roll_pitch_gain_i;
extern float angle_roll_pitch_gain_d;

extern float added_angle_roll_pitch_master_gain;
extern float added_angle_roll_pitch_gain_p;
extern float added_angle_roll_pitch_gain_i;
extern float added_angle_roll_pitch_gain_d;

extern float acro_roll_pitch_master_gain;
extern float acro_roll_pitch_gain_p;
extern float acro_roll_pitch_gain_i;
extern float acro_roll_pitch_gain_d;

float added_acro_roll_pitch_master_gain;
float added_acro_roll_pitch_gain_p;
float added_acro_roll_pitch_gain_i;
float added_acro_roll_pitch_gain_d;

extern const float acro_yaw_gain_master;
extern const float acro_yaw_gain_p;
extern const float acro_yaw_gain_i;

extern float altitude_hold_master_gain;
extern float altitude_hold_gain_p;
extern float altitude_hold_gain_i;
extern float altitude_hold_gain_d;

extern float added_altitude_hold_master_gain;
extern float added_altitude_hold_gain_p;
extern float added_altitude_hold_gain_i;
extern float added_altitude_hold_gain_d;

extern float gps_hold_master_gain;
extern float gps_hold_gain_p;
extern float gps_hold_gain_i;
extern float gps_hold_gain_d;

extern float added_gps_hold_master_gain;
extern float added_gps_hold_gain_p;
extern float added_gps_hold_gain_i;
extern float added_gps_hold_gain_d;

extern float acro_target_roll;
extern float acro_target_pitch;
extern float acro_target_yaw;

extern float angle_target_pitch;
extern float angle_target_roll;

extern float target_altitude_barometer;
extern uint8_t target_altitude_barometer_set;
extern float last_throttle_deadzone;

extern float altitude_barometer_rate_of_change_max_cm_s;
extern float target_longitude;
extern float target_latitude;

extern uint8_t target_lon_lat_set;

extern const float gps_pid_angle_of_attack_max;

extern float acro_mode_rate_degrees_per_second;
extern float angle_mode_rate_degrees_per_second_max_integral_derivative;


extern float gyro_angular_raw[];
extern float acceleration_data_raw[];
extern float magnetometer_data_raw[];
extern float magnetometer_data_secondary_raw[];
extern float magnetometer_data_ist8310_raw[];

extern float altitude_barometer_raw;
extern float acceleration_data_previous[];
extern float magnetometer_data_ist8310[];

extern float magnetometer_data_90[];
extern float magnetometer_data_180[];
extern float magnetometer_data_270[];

extern float acceleration_data[];
extern float gyro_angular[];
extern float imu_orientation[];
extern float old_imu_orientation[];
extern float magnetometer_data[];
extern float magnetometer_data_secondary[];

extern float magnetometer_data_unrotated[];
extern float magnetometer_data_secondary_unrotated[];

extern float gyro_yaw_old;

extern float magnetometer_data_current_unfiltered[];
extern float magnetometer_low_pass[];

extern float gps_yaw;
extern uint8_t gps_fix_type;
extern uint8_t gps_satellites_count;
extern uint8_t got_gps;

extern uint8_t gps_can_be_used;
extern float gps_latitude;
extern float gps_longitude;
extern float gps_height_above_geoid_kilometers;

extern uint8_t gps_date_day;
extern uint8_t gps_date_month;
extern uint16_t gps_date_year;

extern float lat_distance_to_target_meters;
extern float lon_distance_to_target_meters;



extern uint8_t got_ist8310_reading;
extern uint32_t magnetometer_reading_timestamp_time_microseconds;
extern uint32_t last_magnetometer_reading_timestamp_microseconds;



extern float pressure_hpa;
extern float temperature_celsius;
extern float altitude_barometer;
extern float altitude_sensor_fusion;
extern uint8_t got_pressure;

extern float vertical_acceleration_old;
extern float vertical_acceleration;

extern float vertical_velocity;


extern uint8_t gps_position_hold_enabled;
extern uint8_t gps_target_set;
extern uint8_t gps_target_unset_logged;

extern uint8_t use_wmm;
extern uint8_t wmm_perform_elements_compute;
extern uint8_t wmm_elements_computed;

extern uint8_t gps_target_unset_cause;
extern float gps_target_unset_roll_value;
extern float gps_target_unset_pitch_value;

extern uint8_t use_gps_reset_count;
extern uint8_t use_gps_reset_count_to_deactivate; 

extern float gps_hold_roll_adjustment_integral;
extern float gps_hold_pitch_adjustment_integral;

extern float gps_hold_roll_adjustment;
extern float gps_hold_pitch_adjustment;

extern float roll_effect_on_lat;
extern float pitch_effect_on_lat;

extern float roll_effect_on_lon;
extern float pitch_effect_on_lon;

extern float error_forward;
extern float error_right;


extern uint32_t last_got_gps_timestamp;
extern uint32_t max_allowed_time_miliseconds_between_got_gps;


extern uint8_t ms5611_which_conversion;
extern uint32_t ms5611_conversion_start_timestamp;
extern uint32_t ms5611_min_pause_after_conversion_initiate_microseconds;
extern uint32_t ms5611_loop_start_timestamp;
extern uint32_t ms5611_set_reference_pressure_after_microseconds_of_loop;
extern uint8_t ms5611_reference_set;

extern uint8_t initial_yaw_set;
extern uint32_t yaw_loop_start_timestamp;
extern uint32_t yaw_set_yaw_after_microseconds_of_loop;
extern float yaw_magnetometer_only_gate_absolute_degrees;
extern float yaw_gyro_only_gate_absolute_degrees;


extern uint8_t qmc5883;
extern uint8_t mmc5603;
extern uint8_t hmc5883;
extern uint8_t bmm350;
extern uint8_t ist8310;

extern uint8_t mpu6050;
extern uint8_t ms5611;
extern uint8_t bn357;
extern uint8_t nrf24;

// 0.05 Drifts a lot
// 0.5 drift a bit
// 1.0 drift a bit
// 0.3 drift a bit
// 0.1 drift a bit more 
// Goal is to minimize drift while keeping as low as possible.

#define COMPLEMENTARY_RATIO_MULTIPLYER_FLIGHT 0.3f
#define COMPLEMENTARY_RATIO_MULTIPLYER_OFF 7.0f
#define COMPLEMENTARY_RATIO_MULTIPLYER_YAW_LOWER_GATE_MORE_GYRO 0.000f
#define COMPLEMENTARY_RATIO_MULTIPLYER_YAW_UPPER_GATE_MORE_MAG 0.001f // Increase until it starts doing edges on bumps
#define COMPLEMENTARY_RATIO_MULTIPLYER_YAW_USER_INPUT_MORE_MORE_MAG 0.01f // Increase until it starts doing edges on bumps

extern float complementary_ratio;

extern float nyquist_with_margin;

extern float motor_low_pass_filter_cutoff;
extern struct low_pass_pt1_filter filter_motor_0;
extern struct low_pass_pt1_filter filter_motor_1;
extern struct low_pass_pt1_filter filter_motor_2;
extern struct low_pass_pt1_filter filter_motor_3;

extern float filtering_magnetometer_cutoff_frequency;
extern struct low_pass_biquad_filter filter_magnetometer_x;
extern struct low_pass_biquad_filter filter_magnetometer_y;
extern struct low_pass_biquad_filter filter_magnetometer_z;

extern struct low_pass_biquad_filter filter_magnetometer_x_secondary;
extern struct low_pass_biquad_filter filter_magnetometer_y_secondary;
extern struct low_pass_biquad_filter filter_magnetometer_z_secondary;

extern struct low_pass_biquad_filter filter_magnetometer_90_x;
extern struct low_pass_biquad_filter filter_magnetometer_90_y;
extern struct low_pass_biquad_filter filter_magnetometer_90_z;

extern struct low_pass_biquad_filter filter_magnetometer_180_x;
extern struct low_pass_biquad_filter filter_magnetometer_180_y;
extern struct low_pass_biquad_filter filter_magnetometer_180_z;

extern struct low_pass_biquad_filter filter_magnetometer_270_x;
extern struct low_pass_biquad_filter filter_magnetometer_270_y;
extern struct low_pass_biquad_filter filter_magnetometer_270_z;


extern struct low_pass_biquad_filter filter_magnetometer_ist8310_x;
extern struct low_pass_biquad_filter filter_magnetometer_ist8310_y;
extern struct low_pass_biquad_filter filter_magnetometer_ist8310_z;

extern uint8_t magnetometer_iir_filter_order;
extern float magnetometer_iir_filter_feedback[];
extern float magnetometer_iir_filter_feedforward[];


extern struct iir_filter iir_filter_magnetometer_x;
extern struct iir_filter iir_filter_magnetometer_y;
extern struct iir_filter iir_filter_magnetometer_z;

extern uint8_t moving_average_magnetometer_size;

extern struct moving_average moving_average_magnetometer_x;
extern struct moving_average moving_average_magnetometer_y;
extern struct moving_average moving_average_magnetometer_z;

extern float outlier_detection_magnetometer_threshold;
extern uint8_t outlier_detection_magnetometer_window_size;
extern struct outlier_detection outlier_detection_magnetometer_x;
extern struct outlier_detection outlier_detection_magnetometer_y;
extern struct outlier_detection outlier_detection_magnetometer_z;

extern float filtering_accelerometer_cutoff_frequency;
extern struct low_pass_biquad_filter biquad_filter_accelerometer_x;
extern struct low_pass_biquad_filter biquad_filter_accelerometer_y;
extern struct low_pass_biquad_filter biquad_filter_accelerometer_z;

extern struct low_pass_pt1_filter filter_gyro_z;

extern const float filter_gyro_z_yaw_cutoff_frequency;

extern float rpm_notch_filter_q_harmonic_1;
extern float rpm_notch_filter_q_harmonic_3;

extern float rpm_notch_filter_min_frequency;

extern struct notch_filter_q gyro_x_notch_filter_harmonic_1_motor_0;
extern struct notch_filter_q gyro_x_notch_filter_harmonic_1_motor_1;
extern struct notch_filter_q gyro_x_notch_filter_harmonic_1_motor_2;
extern struct notch_filter_q gyro_x_notch_filter_harmonic_1_motor_3;

extern struct notch_filter_q gyro_y_notch_filter_harmonic_1_motor_0;
extern struct notch_filter_q gyro_y_notch_filter_harmonic_1_motor_1;
extern struct notch_filter_q gyro_y_notch_filter_harmonic_1_motor_2;
extern struct notch_filter_q gyro_y_notch_filter_harmonic_1_motor_3;


extern struct notch_filter_q gyro_x_notch_filter_harmonic_3_motor_0;
extern struct notch_filter_q gyro_x_notch_filter_harmonic_3_motor_1;
extern struct notch_filter_q gyro_x_notch_filter_harmonic_3_motor_2;
extern struct notch_filter_q gyro_x_notch_filter_harmonic_3_motor_3;

extern struct notch_filter_q gyro_y_notch_filter_harmonic_3_motor_0;
extern struct notch_filter_q gyro_y_notch_filter_harmonic_3_motor_1;
extern struct notch_filter_q gyro_y_notch_filter_harmonic_3_motor_2;
extern struct notch_filter_q gyro_y_notch_filter_harmonic_3_motor_3;

extern struct low_pass_biquad_filter roll_d_term_filtering;
extern struct low_pass_biquad_filter pitch_d_term_filtering;

extern float gyro_angular_for_d_term[];

extern const float d_term_filtering_min_cutoff;
extern const float d_term_filtering_max_cutoff;
extern const uint8_t d_term_filtering_expo;

extern struct kalman_filter altitude_and_velocity_kalman;

extern const float altitude_barometer_filtering_min_cutoff;
extern struct low_pass_biquad_filter altitude_barometer_filtering;

extern const float gps_filtering_min_cutoff;
extern struct low_pass_pt1_filter lowpass_filter_gps_lat;
extern struct low_pass_pt1_filter lowpass_filter_gps_lon;

extern const float gps_forward_right_filtering_min_cutoff;

extern struct low_pass_pt1_filter lowpass_filter_gps_forward;
extern struct low_pass_pt1_filter lowpass_filter_gps_right;

extern float max_yaw_attack;
extern float max_roll_attack;
extern float max_pitch_attack;
extern float roll_attack_step;
extern float pitch_attack_step;
extern float max_throttle_vertical_velocity;

extern float max_angle_before_motors_off;

extern float throttle;
extern float yaw;
extern float last_yaw;
extern float pitch;
extern float roll;

extern float minimum_signal_timing_seconds;
extern uint64_t last_signal_timestamp_microseconds;

extern uint8_t pid_change_mode;


extern uint8_t log_file_base_name[];
extern uint8_t log_file_base_name_gps[];
extern uint8_t log_file_base_name_alt[];
extern uint8_t log_file_base_name_mag[];
extern uint8_t log_file_base_name_compassRPM[];
extern uint8_t log_file_base_name_maga[];
extern uint8_t log_file_base_name_yaw[];
extern uint8_t log_file_base_name_timing[];
extern uint8_t log_file_base_name_imu[];
extern uint8_t log_file_blackbox_base_name[];
extern uint8_t *new_log_file_blackbox_base_name;
extern uint16_t blackbox_file_index;

extern const uint8_t blackbox_debug_mode;
extern const uint8_t blackbox_log_angle_mode_pid;
extern const uint8_t blackbox_write_repeat_logs_in_same_file;

extern uint8_t sd_card_initialized;
extern uint8_t sd_card_async_initialized;

extern const uint8_t use_simple_async;
extern const uint8_t use_blackbox_logging;

extern uint8_t txt_logging_mode;

extern uint8_t perform_log_for_log_mode_10;

extern uint8_t txt_logged_header;

extern uint32_t loop_iteration;
extern float PID_proportional[];
extern float PID_integral[];
extern float PID_derivative[];
extern float PID_feed_forward[];
extern float PID_set_points[];
extern float remote_control[];

extern uint8_t flight_mode;

extern uint8_t use_disable_all_control;
extern uint8_t use_angle_mode;
extern uint8_t use_vertical_velocity_control;
extern uint8_t use_gps_hold;


extern uint32_t motor_off_index;

extern float yaw_alpha;

extern uint8_t use_compass_rpm;

extern double compass_rpm_mag_1_a;
extern double compass_rpm_mag_1_b;
extern double compass_rpm_mag_1_c;

extern double compass_rpm_mag_2_a;
extern double compass_rpm_mag_2_b;
extern double compass_rpm_mag_2_c;

extern const float compass_frequency_min;
extern const float compass_frequency_max;

extern const uint8_t compass_frequency_values_amount;

extern const float compass_frequency_frequency_samples[];
extern const float compass_frequency_mag1_x_offsets[];
extern const float compass_frequency_mag1_y_offsets[];
extern const float compass_frequency_mag1_z_offsets[];
extern const float compass_frequency_mag2_x_offsets[];
extern const float compass_frequency_mag2_y_offsets[];
extern const float compass_frequency_mag2_z_offsets[];

extern float average_rpm;
extern float mag1_x_offset;
extern float mag1_y_offset;
extern float mag1_z_offset;
extern float mag2_x_offset;
extern float mag2_y_offset;
extern float mag2_z_offset;


#define MICROSECONDS_TO_HOURS (1.0f / 3600000000.0f) // The division is more efficient with multiplication
#define HOURS_TO_MICROSECONDS (3600000000.0f)

#define MICROSECONDS_TO_MINUTES (1.0f / 60000000.0f)
#define MINUTES_TO_MICROSECONDS (60000000.0f)

#define MICROSECONDS_TO_SECONDS (1.0f / 1000000.0f)
#define SECONDS_TO_MICROSECONDS (1000000.0f)

#define MICROSECONDS_TO_MILISECONDS (1.0f / 1000.0f)
#define MILISECONDS_TO_MICROSECONDS (1000.0f)
