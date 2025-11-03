#include "system.h"
#include "../lib/.layers/common.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include <inttypes.h>

// ------------------------------------------------------------------------------------------------------ Layers
#include "../lib/.layers/radio/radio.h"
#include "../lib/.layers/motors/motors.h"
#include "../lib/.layers/sensors/sensors.h"
#include "../lib/.layers/logging/logging.h"
#include "../lib/.layers/startup/startup.h"
#include "../lib/.layers/time_keeping/time_keeping.h"

// bdshot600 pins
// PA8  - TIM1_CH1 for motor back-left (BL)
// PA11 - TIM1_CH4 for motor back right (BR)
// PA1  - TIM2_CH2 for motor front left (FL)
// PB10 - TIM2_CH3 for motor front right (FR)

// UART pins
// PA2  UART2 tx GPS
// PA3  UART2 rx GPS
// PA10 UART1 rx FTDI
// PA9  UART1 tx FTDI

// I2C pins
// PB7  I2C1 SDA
// PB6  I2C1 SCL

// GPIO
// PC13 Internal LED
// PA12 LED

// SPI3 SD card logger
// PB5 MOSI
// PB4 MISO
// PB3 SCK
// PA15 CS
// PA12 Slave ready. Falling interrupt

// SPI1 Radio module
// PA5 SCK
// PA6 MISO
// PA7 MOSI
// PB1 CS 
// PB0 CE

// DMAs used. They have to not be reused otherwise problems
// TIM2_CH3 DMA1 stream 1
// RX UART2 DMA1 stream 5
// TIM2_CH2 DMA1 stream 6
// TX SPI3  DMA1 stream 7
// TIM1_CH1 DMA2 stream 1
// TIM1_CH4 DMA2 stream 4

// Measuring performance
uint32_t radio = 0;
uint32_t radio2 = 0;
uint32_t radio_end = 0;

uint32_t sensors = 0;
uint32_t sensors2 = 0;
uint32_t sensors3 = 0;
uint32_t sensors4 = 0;
uint32_t sensors5 = 0;
uint32_t sensors6 = 0;
uint32_t sensors7 = 0;
uint32_t sensors8 = 0;
uint32_t sensors9 = 0;
uint32_t sensors10 = 0;
uint32_t sensors11 = 0;
uint32_t sensors_end = 0;

uint32_t motor = 0;
uint32_t motor2 = 0;
uint32_t motor3 = 0;
uint32_t motor4 = 0;
uint32_t motor5 = 0;
uint32_t motor6 = 0;
uint32_t motor7 = 0;
uint32_t motor_end = 0;

uint32_t logging = 0;
uint32_t logging2 = 0;
uint32_t logging3 = 0;
uint32_t logging_end = 0;


uint32_t interrupt = 0;
uint32_t interrupt_end = 0;

// ------------------------------------------------------------------------------------------------------ Refresh rate
const uint16_t precalculated_timing_miliseconds = (1000000 / REFRESH_RATE_HZ);

// ------------------------------------------------------------------------------------------------------ handling loop timing
// Timing
// TODO: Change this variable to uint64 later
uint64_t absolute_microseconds_since_start = 0;
uint64_t loop_start_timestamp_microseconds = 0;
uint64_t delta_time_without_waiting_from_previous_loop_microseconds = 0;
uint64_t delta_time_total_loop_time_previous_loop_microseconds = 0;


uint16_t loop_start_microseconds = 0; // The timer is only 16 bits
uint16_t loop_start_miliseconds = 0; // Fallback if the microseconds overflew

// Keep track of time in each loop. Since loop start
uint32_t startup_time_microseconds = 0;
uint32_t delta_time = 0;
uint8_t time_since_startup_hours = 0;
uint8_t time_since_startup_minutes = 0;
uint8_t time_since_startup_seconds = 0;
uint16_t time_since_startup_ms = 0;
uint16_t time_since_startup_microseconds;

uint8_t entered_loop = 0;

// ------------------------------------------------------------------------------------------------------ Motor settings
const uint32_t dshot_refresh_rate = 500; // Hz

// Take 75 percent of max because my battery can't handle all that current.
const uint16_t actual_max_dshot600_throttle_value = min_dshot600_throttle_value + ((max_dshot600_throttle_value - min_dshot600_throttle_value) * 50) / 100; // (max lipo amp rating / max draw of a bldc motor being used x 4) 0.917 * (max_pwm - min_pwm) + min_pwm = 371.4

// Motor 0 - BL, 1 - BR, 2 - FR, 3 - FL
float motor_power[] = {0.0, 0.0, 0.0, 0.0}; // Percent
float motor_frequency[] = {0.0, 0.0, 0.0, 0.0};
float motor_rpm[] = {0.0, 0.0, 0.0, 0.0};

float throttle_value = 0.0; 
uint8_t motor_rotor_poles = 14; // it is the 14 value from 12N14P of ECO II 2807

// Motor handles
uint8_t motor_BL;
uint8_t motor_BR;
uint8_t motor_FR;
uint8_t motor_FL;

// For calculations
uint16_t throttle_value_FR;
uint16_t throttle_value_FL;
uint16_t throttle_value_BL;
uint16_t throttle_value_BR;

// If REFRESH_RATE_HZ >= 500 HZ then the control loop will handle the bdshot600 sending
// else the interrupt on timer 3 will handle it
uint8_t manual_bdshot = 0;


// ------------------------------------------------------------------------------------------------------ Sensor calibrations

// ===================================== Magnetometer soft and hard irons

// Magnetometer MMC5603 field
float magnetometer_mmc5603_hard_iron_correction[3] = {3277.283060f, 3274.944050f, 3270.911984f};
float magnetometer_mmc5603_soft_iron_correction[3][3] = {
  {1.065294f, 0.001482f, -0.003566f},
  {0.001482f, 1.038720f, 0.068150f},
  {-0.003566f, 0.068150f, 1.107906f},
};

float magnetometer_mmc5603_rotation_correction[3][3] = {
  {-0.004702f, 0.998760f, 0.049552f},
  {-0.999989f, -0.004682f, -0.000524f},
  {-0.000291f, -0.049554f, 0.998771f},
};






// Magnetometer BMM350 at field far old
float magnetometer_hard_iron_correction[3] = {-22.253065f, -2.751542f, -7.204707f};
float magnetometer_soft_iron_correction[3][3] = {
  {1.049370f, -0.008414f, -0.007085f},
  {-0.008414f, 1.066816f, -0.012539f},
  {-0.007085f, -0.012539f, 1.124578f},
};

float magnetometer_rotation_correction[3][3] = {
  {0.012639f, 0.999794f, -0.015868f},
  {-0.999759f, 0.012920f, 0.017748f},
  {0.017950f, 0.015640f, 0.999717f},
};







// ist8310 at field
float magnetometer_ist8310_hard_iron_correction[3] = {
    -5.731186, -16.150355, 3.217410
};

float magnetometer_ist8310_soft_iron_correction[3][3] = {
    {1.064339, -0.001697, 0.019543},
    {-0.001697, 1.036534, 0.016589},
    {0.019543, 0.016589, 0.969919}
};



// ===================================== Accelerometer calibration


float accelerometer_correction[3] = {
    0.057752, -0.016512, 0.980417
};

float accelerometer_scale_factor_correction[3][3] = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
};

// ===================================== Accelerometer calibration

// Gyro
float gyro_correction[3] = {
    -2.532809, 2.282250, 0.640916
};

// ===================================== Sensor fusion offsets

// TODO: rename this to something not related to accelerometer
float base_accelerometer_roll_offset = -1.2 - -0.9825;
float base_accelerometer_pitch_offset = -0.76 - -0.8275;

float accelerometer_roll_offset = 0;
float accelerometer_pitch_offset = 0;

float yaw_offset1 = -180.0;
float yaw_offset2 = -90.0;

float yaw_declination = 5.117134f;

// ------------------------------------------------------------------------------------------------------ Accelerometer values to degrees conversion
float accelerometer_roll = 0;
float accelerometer_pitch = 0;

float gyro_yaw = 0;
float magnetometer_yaw_unrotated_no_tilt = 0;
float magnetometer_yaw_unrotated_tilt = 0;
float magnetometer_yaw_no_tilt = 0;
float magnetometer_yaw = 0;
float magnetometer_yaw_secondary = 0;

float magnetometer_yaw_90 = 0;
float magnetometer_yaw_180 = 0;
float magnetometer_yaw_270 = 0;


float magnetometer_yaw_unfiltered_no_tilt = 0;

float magnetometer_ist8310_yaw_unrotated_no_tilt = 0;
float magnetometer_ist8310_yaw_unrotated_tilt = 0;
float magnetometer_ist8310_yaw_no_tilt = 0;
float magnetometer_ist8310_yaw = 0;
float magnetometer_ist8310_yaw_unfiltered_no_tilt = 0;

float magnetometer_data_unfiltered[3] = {0,0,0};
float magnetometer_data_ist8310_unfiltered[3] = {0,0,0};


// ------------------------------------------------------------------------------------------------------ Radio config
uint8_t tx_address[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
char rx_data[32];
char rx_type[32];

// ------------------------------------------------------------------------------------------------------ PID stuff
struct pid acro_roll_pid;
struct pid acro_pitch_pid;
struct pid acro_yaw_pid;

struct pid angle_pitch_pid;
struct pid angle_roll_pid;

struct pid altitude_hold_pid;

struct pid gps_longitude_pid;
struct pid gps_latitude_pid;

float error_acro_roll = 0;
float error_acro_pitch = 0;
float error_acro_yaw = 0;

float error_angle_pitch = 0;
float error_angle_roll = 0;

float error_throttle = 0;
float error_vertical_velocity = 0;
float error_altitude_barometer = 0;

float error_latitude = 0;
float error_longitude = 0;
float last_error_roll = 0;
float last_error_pitch = 0;


// Used for smooth changes to PID while using remote control. Do not touch this
float angle_roll_pitch_master_gain = BASE_ANGLE_roll_pitch_MASTER_GAIN;
float angle_roll_pitch_gain_p = BASE_ANGLE_roll_pitch_GAIN_P;
float angle_roll_pitch_gain_i = BASE_ANGLE_roll_pitch_GAIN_I;
float angle_roll_pitch_gain_d = BASE_ANGLE_roll_pitch_GAIN_D;

float added_angle_roll_pitch_master_gain = 0.0f;
float added_angle_roll_pitch_gain_p = 0.0f;
float added_angle_roll_pitch_gain_i = 0.0f;
float added_angle_roll_pitch_gain_d = 0.0f;

float acro_roll_pitch_master_gain = BASE_ACRO_roll_pitch_MASTER_GAIN;
float acro_roll_pitch_gain_p = BASE_ACRO_roll_pitch_GAIN_P;
float acro_roll_pitch_gain_i = BASE_ACRO_roll_pitch_GAIN_I;
float acro_roll_pitch_gain_d = BASE_ACRO_roll_pitch_GAIN_D;

float added_acro_roll_pitch_master_gain = 0.0f;
float added_acro_roll_pitch_gain_p = 0.0f;
float added_acro_roll_pitch_gain_i = 0.0f;
float added_acro_roll_pitch_gain_d = 0.0f;

// PID for acro yaw
const float acro_yaw_gain_master = 4.0;
const float acro_yaw_gain_p = 0.45;
const float acro_yaw_gain_i = 0.3;

// PID for altitude control
float altitude_hold_master_gain = BASE_ALTITUDE_HOLD_MASTER_GAIN;
float altitude_hold_gain_p = BASE_ALTITUDE_HOLD_GAIN_P;
float altitude_hold_gain_i = BASE_ALTITUDE_HOLD_GAIN_I;
float altitude_hold_gain_d = BASE_ALTITUDE_HOLD_GAIN_D;

float added_altitude_hold_master_gain = 0.0f;
float added_altitude_hold_gain_p = 0.0f;
float added_altitude_hold_gain_i = 0.0f;
float added_altitude_hold_gain_d = 0.0f;

// PID for gps hold
float gps_hold_master_gain = BASE_GPS_HOLD_MASTER_GAIN;
float gps_hold_gain_p = BASE_GPS_HOLD_GAIN_P;
float gps_hold_gain_i = BASE_GPS_HOLD_GAIN_I;
float gps_hold_gain_d = BASE_GPS_HOLD_GAIN_D;

float added_gps_hold_master_gain = 0.0f;
float added_gps_hold_gain_p = 0.0f;
float added_gps_hold_gain_i = 0.0f;
float added_gps_hold_gain_d = 0.0f;

float acro_target_roll = 0.0;
float acro_target_pitch = 0.0;
float acro_target_yaw = 0.0;

float angle_target_pitch = 0.0;
float angle_target_roll = 0.0;

float target_altitude_barometer = 0.0;
uint8_t target_altitude_barometer_set = 0;
float last_throttle_deadzone = 0.0;

float altitude_barometer_rate_of_change_max_cm_s = 30.0f;
float target_longitude = 0.0;
float target_latitude = 0.0;

uint8_t target_lon_lat_set = 0;

// ------------------------------------------------------------------------------------------------------ Some configurations relating to PID

float acro_mode_rate_degrees_per_second = 100;
float angle_mode_rate_degrees_per_second_max_integral_derivative = 3; // At max the integral can accumulate 3 degrees per second of rotation. This is to prevent overshooting

// ------------------------------------------------------------------------------------------------------ Sensor other data

// Raw versions of data
float gyro_angular_raw[] = {0,0,0};
float acceleration_data_raw[] = {0,0,0};
float magnetometer_data_raw[] = {0,0,0};
float magnetometer_data_secondary_raw[] = {0,0,0};
float magnetometer_data_ist8310_raw[] = {0,0,0};

float altitude_barometer_raw = 0.0f;
float acceleration_data_previous[] = {0,0,0};
float magnetometer_data_ist8310[] = {0,0,0};

float magnetometer_data_90[] = {0,0,0};
float magnetometer_data_180[] = {0,0,0};
float magnetometer_data_270[] = {0,0,0};
 
// In use versions of data
float acceleration_data[] = {0,0,0};
float gyro_angular[] = {0,0,0};
float imu_orientation[] = {0,0,0};
float old_imu_orientation[] = {0,0,0};
float magnetometer_data[] = {0,0,0};
float magnetometer_data_secondary[] = {0,0,0};

float magnetometer_data_unrotated[3] = {0.0f, 0.0f, 0.0f};
float magnetometer_data_secondary_unrotated[3] = {0.0f, 0.0f, 0.0f};



float gyro_yaw_old = 0.0f; // For testing yaw

float magnetometer_data_current_unfiltered[] = {0,0,0}; // For testing yaw
float magnetometer_low_pass[] = {0,0,0}; // For testing yaw


// Barometer ist8310 reading timing
uint8_t got_ist8310_reading = 0;
uint32_t magnetometer_reading_timestamp_time_microseconds = 5000;
uint32_t last_magnetometer_reading_timestamp_microseconds = 0;


// Barometer data derivatives
float pressure_hpa = 0.0;
float temperature_celsius = 0.0;
float altitude_barometer = 0.0; // centimeters
float altitude_sensor_fusion = 0.0; // centimeters
uint8_t got_pressure = 0;

float vertical_acceleration_old = 0.0; // UNITS???
float vertical_acceleration = 0.0; // UNITS???

float vertical_velocity = 0.0; // UNITS???


// Gps position hold data
const float gps_pid_angle_of_attack_max = 9.0f;

float gps_yaw = 0;
uint8_t gps_fix_type = 0;
uint8_t gps_satellites_count = 0;
uint8_t got_gps = 0;

uint8_t gps_can_be_used = 0;
float gps_latitude = 0.0;
float gps_longitude = 0.0;
float gps_height_above_geoid_kilometers = 0.0;

uint8_t gps_date_day = 0;
uint8_t gps_date_month = 0;
uint16_t gps_date_year = 0;


float lat_distance_to_target_meters = 0.0;
float lon_distance_to_target_meters = 0.0;

uint8_t gps_position_hold_enabled = 0;
uint8_t gps_target_set = 0;
uint8_t gps_target_unset_logged = 1;

uint8_t use_wmm = 1;
uint8_t wmm_perform_elements_compute = 1;
uint8_t wmm_elements_computed = 0;




// 0 - pitch
// 1 - roll 
// 2 - radio off
uint8_t gps_target_unset_cause = 1; // For logging
float gps_target_unset_roll_value = 0.0f;
float gps_target_unset_pitch_value = 0.0f;

uint8_t use_gps_reset_count = 0;
uint8_t use_gps_reset_count_to_deactivate = 10; // This is done because sometimes some noise sends a neutural pitch and roll

float gps_hold_roll_adjustment_integral = 0.0;
float gps_hold_pitch_adjustment_integral = 0.0;

float gps_hold_roll_adjustment = 0.0f;
float gps_hold_pitch_adjustment = 0.0f;

float roll_effect_on_lat = 0;
float pitch_effect_on_lat = 0;

float roll_effect_on_lon = 0;
float pitch_effect_on_lon = 0;

// new method to control gps
float error_forward = 0.0f;
float error_right = 0.0f;


uint32_t last_got_gps_timestamp = 0;
uint32_t max_allowed_time_miliseconds_between_got_gps = 1000;


// For ms5611 timing
uint8_t ms5611_which_conversion = 0;// 0 - temperature, 1 - pressure
uint32_t ms5611_conversion_start_timestamp = 0;
uint32_t ms5611_min_pause_after_conversion_initiate_microseconds = 0; // Ask the driver what how many microseconds it needs to wait after initiating a conversion
uint32_t ms5611_loop_start_timestamp = 0;
uint32_t ms5611_set_reference_pressure_after_microseconds_of_loop = 4000000; // The actual pressure reading is only available some time into the loo. I do not know why.
uint8_t ms5611_reference_set = 0;

// For yaw of magnetometer/gyro 
uint8_t initial_yaw_set = 0;
uint32_t yaw_loop_start_timestamp = 0;
uint32_t yaw_set_yaw_after_microseconds_of_loop = 1000000; // The actual yaw reading is only available some time into the loop. I do not know why.
float yaw_magnetometer_only_gate_absolute_degrees = 0.1f; // Only magnetometer used bellow this value
float yaw_gyro_only_gate_absolute_degrees = 2.0f; // Only gyro used above this value


// ------------------------------------------------------------------------------------------------------ Sensor initialization status
uint8_t qmc5883 = 0;
uint8_t mmc5603 = 0;
uint8_t hmc5883 = 0;
uint8_t bmm350 = 0;
uint8_t ist8310 = 0;

uint8_t mpu6050 = 0;
uint8_t ms5611 = 0;
uint8_t bn357 = 0;
uint8_t nrf24 = 0;
// ------------------------------------------------------------------------------------------------------ Sensor fusion and filtering stuff

float complementary_ratio = COMPLEMENTARY_RATIO_MULTIPLYER_OFF * (1.0 - 1.0/(1.0+(1.0/REFRESH_RATE_HZ)));

// This is used for filtering limits
float nyquist_with_margin = REFRESH_RATE_HZ / 2.0f - 6.0f; // The -6 is the margin that greatly improves the quality of filtering for notch filters

// ============================= Filter motor frequencies
float motor_low_pass_filter_cutoff = 150;
struct low_pass_pt1_filter filter_motor_0;
struct low_pass_pt1_filter filter_motor_1;
struct low_pass_pt1_filter filter_motor_2;
struct low_pass_pt1_filter filter_motor_3;

// ============================= Magnetometer 
float filtering_magnetometer_cutoff_frequency = 10;
struct low_pass_biquad_filter filter_magnetometer_x;
struct low_pass_biquad_filter filter_magnetometer_y;
struct low_pass_biquad_filter filter_magnetometer_z;

struct low_pass_biquad_filter filter_magnetometer_x_secondary;
struct low_pass_biquad_filter filter_magnetometer_y_secondary;
struct low_pass_biquad_filter filter_magnetometer_z_secondary;

struct low_pass_biquad_filter filter_magnetometer_90_x;
struct low_pass_biquad_filter filter_magnetometer_90_y;
struct low_pass_biquad_filter filter_magnetometer_90_z;

struct low_pass_biquad_filter filter_magnetometer_180_x;
struct low_pass_biquad_filter filter_magnetometer_180_y;
struct low_pass_biquad_filter filter_magnetometer_180_z;

struct low_pass_biquad_filter filter_magnetometer_270_x;
struct low_pass_biquad_filter filter_magnetometer_270_y;
struct low_pass_biquad_filter filter_magnetometer_270_z;


struct low_pass_biquad_filter filter_magnetometer_ist8310_x;
struct low_pass_biquad_filter filter_magnetometer_ist8310_y;
struct low_pass_biquad_filter filter_magnetometer_ist8310_z;

// Cutoff 20Hz sampling 520Hz
uint8_t magnetometer_iir_filter_order = 4;
float magnetometer_iir_filter_feedback[] = {1.0, -3.369238, 4.29911246, -2.45817706, 0.53083649}; // A
float magnetometer_iir_filter_feedforward[] = {0.00015837, 0.00063348, 0.00095021, 0.00063348, 0.00015837}; // B


struct iir_filter iir_filter_magnetometer_x;
struct iir_filter iir_filter_magnetometer_y;
struct iir_filter iir_filter_magnetometer_z;

// uint8_t moving_average_magnetometer_size = 8; // Gives a cutoff of about 9
uint8_t moving_average_magnetometer_size = 32; // Gives a cutoff of about 9

struct moving_average moving_average_magnetometer_x;
struct moving_average moving_average_magnetometer_y;
struct moving_average moving_average_magnetometer_z;

float outlier_detection_magnetometer_threshold = 3.0f;
uint8_t outlier_detection_magnetometer_window_size = 10;
struct outlier_detection outlier_detection_magnetometer_x;
struct outlier_detection outlier_detection_magnetometer_y;
struct outlier_detection outlier_detection_magnetometer_z;

//  ============================= Accelerometer
float filtering_accelerometer_cutoff_frequency = 15;
struct low_pass_biquad_filter biquad_filter_accelerometer_x;
struct low_pass_biquad_filter biquad_filter_accelerometer_y;
struct low_pass_biquad_filter biquad_filter_accelerometer_z;

// ============================= Gyro z
struct low_pass_pt1_filter filter_gyro_z;

// ============================= Gyro filter based on motor frequencies
const float filter_gyro_z_yaw_cutoff_frequency = 50; // 100 Hz was fine also

// Do not go higher in q than 1000
float rpm_notch_filter_q_harmonic_1 = 500;
float rpm_notch_filter_q_harmonic_3 = 200; // Stronger than the 1st harmonic

float rpm_notch_filter_min_frequency = 10; // Hz

struct notch_filter_q gyro_x_notch_filter_harmonic_1_motor_0;
struct notch_filter_q gyro_x_notch_filter_harmonic_1_motor_1;
struct notch_filter_q gyro_x_notch_filter_harmonic_1_motor_2;
struct notch_filter_q gyro_x_notch_filter_harmonic_1_motor_3;

struct notch_filter_q gyro_y_notch_filter_harmonic_1_motor_0;
struct notch_filter_q gyro_y_notch_filter_harmonic_1_motor_1;
struct notch_filter_q gyro_y_notch_filter_harmonic_1_motor_2;
struct notch_filter_q gyro_y_notch_filter_harmonic_1_motor_3;

// 2nd harmonic is not used as its not visible in the logs

struct notch_filter_q gyro_x_notch_filter_harmonic_3_motor_0;
struct notch_filter_q gyro_x_notch_filter_harmonic_3_motor_1;
struct notch_filter_q gyro_x_notch_filter_harmonic_3_motor_2;
struct notch_filter_q gyro_x_notch_filter_harmonic_3_motor_3;

struct notch_filter_q gyro_y_notch_filter_harmonic_3_motor_0;
struct notch_filter_q gyro_y_notch_filter_harmonic_3_motor_1;
struct notch_filter_q gyro_y_notch_filter_harmonic_3_motor_2;
struct notch_filter_q gyro_y_notch_filter_harmonic_3_motor_3;
// Z axis is filtered in other way

// ============================= Gyro filtering for d term of PID (should say x and y not roll and pitch)
struct low_pass_biquad_filter roll_d_term_filtering;
struct low_pass_biquad_filter pitch_d_term_filtering;

// For both roll and pitch gyro
float gyro_angular_for_d_term[] = {0,0,0};
const float d_term_filtering_min_cutoff = 75;
const float d_term_filtering_max_cutoff = 100;
const uint8_t d_term_filtering_expo = 7;

// ============================= Barometer
struct kalman_filter altitude_and_velocity_kalman;

const float altitude_barometer_filtering_min_cutoff = 2;
struct low_pass_biquad_filter altitude_barometer_filtering;

// ============================= GPS
const float gps_filtering_min_cutoff = 3.0f;
// struct low_pass_biquad_filter biquad_filter_gps_lat;
// struct low_pass_biquad_filter biquad_filter_gps_lon;
struct low_pass_pt1_filter lowpass_filter_gps_lat;
struct low_pass_pt1_filter lowpass_filter_gps_lon;

const float gps_forward_right_filtering_min_cutoff = 10.0f;
struct low_pass_pt1_filter lowpass_filter_gps_forward;
struct low_pass_pt1_filter lowpass_filter_gps_right;


// ------------------------------------------------------------------------------------------------------ Remote control settings
float max_yaw_attack = 80.0;
float max_roll_attack = 20;
float max_pitch_attack = 20;
float roll_attack_step = 0.2;
float pitch_attack_step = 0.2;
float max_throttle_vertical_velocity = 15; // cm/second

float max_angle_before_motors_off = 40;

float throttle = 0.0;
float yaw = 50.0;
float last_yaw = 50.0;
float pitch = 50.0;
float roll = 50.0;

float minimum_signal_timing_seconds = 0.2;
uint64_t last_signal_timestamp_microseconds = 0;

// Changing pid changes which pid
// 0 - Acro mode
// 1 - angle mode
// 2 - all depending on flight mode set
//      flight mode 1 - acro mode pid
//      flight mode 2 - angle mode pid
//      flight mode 3 - altitude hold pid
//      flight mode 4 - gps hold pid
//      flight mode 5 - gps hold pid
uint8_t pid_change_mode = 2;



// ------------------------------------------------------------------------------------------------------ Logging to SD card
uint8_t log_file_base_name[] = "Quad.txt";
uint8_t log_file_base_name_gps[] = "QuadGPS.csv";
uint8_t log_file_base_name_alt[] = "QuadALT.csv";
uint8_t log_file_base_name_mag[] = "QuadMAG.txt";
uint8_t log_file_base_name_compassRPM[] = "QuadCompassRPM.csv";
uint8_t log_file_base_name_maga[] = "QuadMAGA.csv";
uint8_t log_file_base_name_yaw[] = "QuadYAW.csv";
uint8_t log_file_base_name_timing[] = "QuadTIMING.csv";
uint8_t log_file_base_name_imu[] = "QuadIMU.csv";
uint8_t log_file_blackbox_base_name[] = "Quadcopter.BBL";
uint8_t *new_log_file_blackbox_base_name;
uint16_t blackbox_file_index = 0;
// Debug modes
// 0 - motor frequency
// 1 - imu data from drone
// 2 - vertical acceleration data instead of acro mode yaw data
const uint8_t blackbox_debug_mode = 2;
// 0 - acro mode
// 1 - angle mode
const uint8_t blackbox_log_angle_mode_pid = 1;
const uint8_t blackbox_write_repeat_logs_in_same_file = 1;

uint8_t sd_card_initialized = 0;
uint8_t sd_card_async_initialized = 0;


// TODO: clarify what this does better
// 0 - complex async (faster)
// 1 - simple async
const uint8_t use_simple_async = 0;
const uint8_t use_blackbox_logging = 0;

// 0 - general logging
// 1 - gps logging
// 2 - vertical velocity logging
// 3 - magnetometer raw data
// 4 - yaw data
// 5 - timing data
// 6 - imu data
// 7 - magnetometer calibrated data
// 8 - compassRPM (like compassMOT but with rpm)
// 9 - magnetometer raw data + accelerometer + gyro
// 10 - magnetometer raw data + accelerometer + gyro BUT TRIGGER BASED
uint8_t txt_logging_mode = 4;

uint8_t perform_log_for_log_mode_10 = 0;

// This lets the logger system know if the header has been printed already
uint8_t txt_logged_header = 0; 

// ------------------------------------------------------------------------------------------------------ Stuff for blackbox logging
uint32_t loop_iteration = 1;
float PID_proportional[5]; // acro roll, acro pitch, acro yaw, altitude hold outer, altitude hold inner
float PID_integral[5]; // acro roll, acro pitch, acro yaw, altitude hold outer, altitude hold inner
float PID_derivative[5]; // acro roll, acro pitch, acro yaw, altitude hold outer, altitude hold inner
float PID_feed_forward[3];
float PID_set_points[6]; // acro roll, acro pitch, acro yaw, throttle, altitude hold outer, altitude hold inner
float remote_control[4];

// ------------------------------------------------------------------------------------------------------ Flight mode settings
// Flight modes:
// (0) - no control just direct throttle
// (1) - acro
// (2) - angle
// (3) - angle with altitude hold
// (4) - angle with altitude hold and gps hold
// (5) - angle with gps hold and without altitude hold
uint8_t flight_mode = 5;

uint8_t use_disable_all_control = 0;
uint8_t use_angle_mode = 0;
uint8_t use_vertical_velocity_control = 0;
uint8_t use_gps_hold = 0;


//Logging motor off reason
uint32_t motor_off_index = 0;


// ------------------------------------------------------------------------------------------------------ Magnetometer testing

float yaw_alpha = 0.0f;

uint8_t use_compass_rpm = 1;

double compass_rpm_mag_1_a = -163842.067952876677737;
double compass_rpm_mag_1_b = 0.000000252619974;
double compass_rpm_mag_1_c = 0.771797640071038;

double compass_rpm_mag_2_a = -1408031.579263158375397;
double compass_rpm_mag_2_b = 0.000000207891335;
double compass_rpm_mag_2_c = 7.716531217817929;

const float compass_frequency_min = 0.0f;
const float compass_frequency_max = 166.575000f;


// mag1 x - 11    167.627500
// mag1 y - all zeros or +0.5 
// mag1 z - 12 166.575000

// mag2 x -10 168.085000
// mag2 y -12 166.575000
// mag2 z -12 166.575000


const uint8_t compass_frequency_values_amount = 100;

const float compass_frequency_frequency_samples[] = {
     0.000000,
    1.682576,
    3.365152,
    5.047727,
    6.730303,
    8.412879,
    10.095455,
    11.778030,
    13.460606,
    15.143182,
    16.825758,
    18.508333,
    20.190909,
    21.873485,
    23.556061,
    25.238636,
    26.921212,
    28.603788,
    30.286364,
    31.968939,
    33.651515,
    35.334091,
    37.016667,
    38.699242,
    40.381818,
    42.064394,
    43.746970,
    45.429545,
    47.112121,
    48.794697,
    50.477273,
    52.159848,
    53.842424,
    55.525000,
    57.207576,
    58.890152,
    60.572727,
    62.255303,
    63.937879,
    65.620455,
    67.303030,
    68.985606,
    70.668182,
    72.350758,
    74.033333,
    75.715909,
    77.398485,
    79.081061,
    80.763636,
    82.446212,
    84.128788,
    85.811364,
    87.493939,
    89.176515,
    90.859091,
    92.541667,
    94.224242,
    95.906818,
    97.589394,
    99.271970,
    100.954545,
    102.637121,
    104.319697,
    106.002273,
    107.684848,
    109.367424,
    111.050000,
    112.732576,
    114.415152,
    116.097727,
    117.780303,
    119.462879,
    121.145455,
    122.828030,
    124.510606,
    126.193182,
    127.875758,
    129.558333,
    131.240909,
    132.923485,
    134.606061,
    136.288636,
    137.971212,
    139.653788,
    141.336364,
    143.018939,
    144.701515,
    146.384091,
    148.066667,
    149.749242,
    151.431818,
    153.114394,
    154.796970,
    156.479545,
    158.162121,
    159.844697,
    161.527273,
    163.209848,
    164.892424,
    166.575000
};

const float compass_frequency_mag1_x_offsets[] ={
    0.000000,
    0.000000,
    0.217936,
    0.252581,
    0.297227,
    0.283227,
    0.347227,
    0.421227,
    0.491227,
    0.517227,
    0.463227,
    0.401227,
    0.395227,
    0.407227,
    0.445227,
    0.509227,
    0.557227,
    0.579227,
    0.577227,
    0.577227,
    0.571227,
    0.571227,
    0.569227,
    0.569227,
    0.561227,
    0.599227,
    0.575227,
    0.559227,
    0.611227,
    0.659227,
    0.631227,
    0.671227,
    0.693227,
    0.661227,
    0.655227,
    0.667227,
    0.675227,
    0.683227,
    0.719227,
    0.739227,
    0.779227,
    0.777227,
    0.791227,
    0.785227,
    0.781227,
    0.805227,
    0.833227,
    0.847227,
    0.841227,
    0.863227,
    0.871227,
    0.897227,
    0.899227,
    0.907227,
    0.901227,
    0.911227,
    0.931227,
    0.973227,
    1.063227,
    1.119227,
    1.139227,
    1.187227,
    1.213227,
    1.199227,
    1.207227,
    1.191227,
    1.243227,
    1.305227,
    1.295227,
    1.329227,
    1.363227,
    1.295227,
    1.269227,
    1.329227,
    1.341227,
    1.381227,
    1.425227,
    1.451227,
    1.475227,
    1.491227,
    1.523227,
    1.565227,
    1.637227,
    1.683227,
    1.769227,
    1.797227,
    1.839227,
    1.873227,
    1.909227,
    1.925227,
    1.979227,
    2.033227,
    2.061227,
    2.095227,
    2.179227,
    2.181227,
    2.193227,
    2.225227,
    2.203227,
    2.333227
};

const float compass_frequency_mag1_y_offsets[] ={
    0.0,
    0.0,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5
};

const float compass_frequency_mag1_z_offsets[] ={
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    -0.146568,
    -0.271137,
    -0.299705,
    -0.414274,
    -0.608842,
    -0.654842,
    -0.594842,
    -0.692842,
    -0.780842,
    -0.732842,
    -0.692842,
    -1.048842,
    -1.110842,
    -1.182842,
    -0.942842,
    -1.372842
};


const float compass_frequency_mag2_x_offsets[] ={
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    -0.087374,
    -0.100747,
    -0.194121,
    -0.255494,
    -0.286868,
    -0.272868,
    -0.346868,
    -0.336868,
    -0.378868,
    -0.434868,
    -0.442868,
    -0.428868,
    -0.458868,
    -0.488868,
    -0.490868,
    -0.528868,
    -0.590868,
    -0.616868,
    -0.624868,
    -0.710868,
    -0.830868,
    -0.906868,
    -0.944868,
    -0.964868,
    -1.030868,
    -1.044868,
    -1.024868,
    -1.098868,
    -1.176868,
    -1.202868,
    -1.294868,
    -1.404868,
    -1.462868,
    -1.472868,
    -1.534868,
    -1.488868,
    -1.474868,
    -1.528868,
    -1.646868,
    -1.688868,
    -1.780868,
    -1.952868,
    -1.994868,
    -2.070868,
    -2.156868,
    -2.286868,
    -2.318868,
    -2.426868,
    -2.562868,
    -2.718868,
    -2.794868,
    -2.938868,
    -3.070868,
    -3.106868,
    -3.228868,
    -3.396868,
    -3.494868,
    -3.676868,
    -3.794868,
    -3.910868,
    -4.040868,
    -4.220868,
    -4.288868,
    -4.602868,
    -4.814868,
    -4.960868,
    -5.212868,
    -5.486868,
    -5.538868,
    -5.602868,
    -5.886868,
    -6.002868,
    -6.232868,
    -6.456868,
    -6.724868,
    -6.840868,
    -7.054868,
    -7.142868,
    -7.376868,
    -7.516868,
    -7.668868,
    -7.676868,
    -7.966868,
};

const float compass_frequency_mag2_y_offsets[] ={
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    -0.038916,
    -0.087833,
    -0.158749,
    -0.239666,
    -0.406582,
    -0.548582,
    -0.614582,
    -0.634582,
    -0.656582,
    -0.612582,
    -0.574582,
    -0.638582,
    -0.706582,
    -0.814582,
    -0.910582,
    -1.002582,
    -1.054582,
    -1.132582,
    -1.174582,
    -1.278582,
    -1.334582,
    -1.422582,
    -1.498582,
    -1.628582,
    -1.716582,
    -1.888582,
    -2.002582,
    -2.154582,
    -2.262582,
    -2.306582,
    -2.352582,
    -2.470582,
    -2.608582,
    -2.724582,
    -2.926582,
    -3.070582,
    -3.220582,
    -3.318582,
    -3.362582,
    -3.398582,
    -3.454582,
    -3.570582,
    -3.698582,
    -3.874582,
    -4.126582,
    -4.406582,
    -4.514582,
    -4.758582,
    -5.106582,
    -5.300582,
    -5.554582,
    -5.876582,
    -6.148582,
    -6.184582,
    -6.388582,
    -6.592582,
    -6.816582,
    -7.010582,
    -7.436582,
    -7.826582,
    -8.074582,
    -8.324582,
    -8.614582,
    -8.778582,
    -8.854582,
    -9.004582,
    -9.044582
};

const float compass_frequency_mag2_z_offsets[] = {
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    0.000000,
    -0.029358,
    -0.088716,
    -0.182074,
    -0.301432,
    -0.460790,
    -0.614790,
    -0.744790,
    -0.826790,
    -0.872790,
    -0.892790,
    -0.850790,
    -0.828790,
    -0.932790,
    -1.054790,
    -1.062790,
    -1.090790,
    -1.012790,
    -0.886790,
    -0.830790,
    -0.934790,
    -1.108790,
    -1.314790,
    -1.474790,
    -1.562790,
    -1.632790,
    -1.754790,
    -1.946790,
    -2.134790,
    -2.362790,
    -2.530790,
    -2.608790,
    -2.564790,
    -2.494790,
    -2.534790,
    -2.604790,
    -2.656790,
    -2.912790,
    -3.196790,
    -3.314790,
    -3.498790,
    -3.690790,
    -3.786790,
    -3.908790,
    -3.938790,
    -4.006790,
    -4.012790,
    -4.104790,
    -4.242790,
    -4.486790,
    -4.694790,
    -5.026790,
    -5.386790,
    -5.628790,
    -5.834790,
    -6.126790,
    -6.388790,
    -6.550790,
    -6.746790,
    -6.978790,
    -7.202790,
    -7.372790,
    -7.444790,
    -7.662790,
    -8.024790,
    -8.224790,
    -8.616790,
    -9.038790,
    -9.432790,
    -9.650790,
    -10.012790,
    -10.252790,
    -10.714790,
    -10.952790,
    -11.372790,
    -11.634790,
    -12.098790,
    -12.350790,
    -12.804790,
    -13.338790,
    -13.888790,
    -14.366790,
    -14.930790,
    -15.450790,
    -15.966790,
    -16.606790,
    -16.978790,
    -17.540790,
    -18.108790,
    -18.298790,
    -19.026790,
    -18.286790
};

float average_rpm = 0.0f;
float mag1_x_offset = 0.0f;
float mag1_y_offset = 0.0f;
float mag1_z_offset = 0.0f;
float mag2_x_offset = 0.0f;
float mag2_y_offset = 0.0f;
float mag2_z_offset = 0.0f;


// ------------------------------------------------------------------------------------------------------ MAIN
int system_main(void){

    if(startup_procedure() == 1){
        return 1;
    }

    handle_pre_loop_start();
    while (1){
        // Get current time
        handle_loop_start();

        handle_radio_communication();
        post_remote_control_step();

        handle_get_and_calculate_sensor_values(); // Important do do this right before the pid stuff.

        handle_pid_and_motor_control();

        handle_logging();

        handle_loop_end();
    }
}