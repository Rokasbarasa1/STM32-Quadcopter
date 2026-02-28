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
#include "../lib/.layers/procedures/procedures.h"
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







// Magnetometer BMM350 at field far old
float magnetometer_hard_iron_correction[3] = {28.415474f, 29.001620f, 5.902326f};
float magnetometer_soft_iron_correction[3][3] = {
  {0.977122f, 0.003435f, 0.040408f},
  {0.003435f, 0.990187f, 0.001663f},
  {0.040408f, 0.001663f, 1.070068f},
};

float magnetometer_rotation_correction[3][3] = {
  {-0.998219f, 0.004707f, 0.059468f},
  {0.001730f, 0.998747f, -0.050015f},
  {-0.059629f, -0.049823f, -0.996976f},
};


// Magnetometer MMC5603 field
float magnetometer_mmc5603_hard_iron_correction[3] = {3286.459643f, 3316.110092f, 3287.954983f};
float magnetometer_mmc5603_soft_iron_correction[3][3] = {
  {0.980429f, 0.000419f, 0.002566f},
  {0.000419f, 0.924786f, -0.020750f},
  {0.002566f, -0.020750f, 1.031637f},
};

float magnetometer_mmc5603_rotation_correction[3][3] = {
  {-0.002094f, 0.999924f, -0.012138f},
  {-0.999465f, -0.001696f, 0.032659f},
  {0.032635f, 0.012200f, 0.999393f},
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

// Corrects for distortions in accelerometer ellipsoid
float accelerometer_ellipsoid_correction[3][3] = {
    {1.010731, -0.000407, 0.106118},
    {-0.000407, 0.999444, -0.003237},
    {-0.063155, 0.119400, 0.974078}
};

// Corects for the accelerometer values not being centered around 0
float accelerometer_offset_correction[3] = {
    0.036075, 0.000005, -0.038030
};

// Level correction
float accelerometer_level_correction[3] = {
    0.125832, -0.015120, 0.989827
};
// ACCELEROMETER errors:  X,Y,Z  0.049042, -0.017588, 0.977090

// ===================================== Accelerometer calibration

// Gyro
float gyro_correction[3] = {
    -2.532809, 2.282250, 0.640916
};

// ===================================== Sensor fusion offsets

// TODO: rename this to something not related to accelerometer
// float base_accelerometer_roll_offset = -1.2 - -0.9825;
// float base_accelerometer_pitch_offset = -0.76 - -0.8275;

float base_accelerometer_roll_offset = -0.768168;
float base_accelerometer_pitch_offset = -0.116167;

float accelerometer_roll_offset = 0;
float accelerometer_pitch_offset = 0;

float yaw_offset1 = -180.0;
float yaw_offset2 = -90.0;

float yaw_declination = 5.117134f + 0.0f;

// ------------------------------------------------------------------------------------------------------ Accelerometer values to degrees conversion
float accelerometer_roll = 0;
float accelerometer_pitch = 0;

float gyro_yaw = 0;
float gyro_yaw2 = 0;

float magnetometer_yaw_unrotated_no_tilt = 0;
float magnetometer_yaw_unrotated_tilt = 0;
float magnetometer_yaw_no_tilt = 0;
float magnetometer_yaw = 0;
float magnetometer_yaw_unrotated = 0;
float magnetometer_yaw_90 = 0;
float magnetometer_yaw_180 = 0;
float magnetometer_yaw_270 = 0;

float magnetometer_yaw_secondary = 0;

float magnetometer_yaw_secondary_unrotated = 0;
float yaw_sensor_fusion = 0;



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


struct pid gps_longitude_outter_pid;
struct pid gps_latitude_outter_pid;

struct pid gps_longitude_slowed_pid;
struct pid gps_latitude_slowed_pid;

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

float gps_hold_yaw_speed_gain_ff = BASE_GPS_HOLD_YAW_SPEED_FF;

float added_gps_hold_master_gain = 0.0f;
float added_gps_hold_gain_p = 0.0f;
float added_gps_hold_gain_i = 0.0f;
float added_gps_hold_gain_d = 0.0f;

float added_gps_hold_yaw_speed_gain_ff = 0.0f;

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

float gps_yaw = 0.0f;
float gps_speed_ms = 0.0f;

float gps_speed_ms_north = 0.0f;
float gps_speed_ms_east = 0.0f;

float vel_sp_north = 0.0f;
float vel_sp_east = 0.0f;

float vel_error_north = 0.0f;
float vel_error_east = 0.0f;



uint8_t gps_fix_type = 0;
uint8_t gps_satellites_count = 0;
float gps_position_accuracy = 0;
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

float gps_hold_roll_adjustment_not_slowed = 0.0f;
float gps_hold_pitch_adjustment_not_slowed = 0.0f;

float roll_effect_on_lat = 0;
float pitch_effect_on_lat = 0;

float roll_effect_on_lon = 0;
float pitch_effect_on_lon = 0;

// new method to control gps
float error_gps_pitch = 0.0f;
float error_gps_roll = 0.0f;

float slowing_forward = 0.0f;
float slowing_right = 0.0f;



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
float filtering_magnetometer_cutoff_frequency = 5;
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
struct low_pass_pt1_filter lowpass_yaw_rad;


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
uint8_t log_file_base_name_raw[] = "QuadRAW.csv";
uint8_t log_file_blackbox_base_name[] = "Quadcopter.BBL";
uint8_t log_file_base_name_raw_gps[] = "QuadRAWGPS.txt";

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
// 11 - raw data and not raw data for learning kalman filter
// 12 - raw gps 
uint8_t txt_logging_mode = 1;

// 0 - GNGGA
// 1 - GNGSA
// 2 - GNRMC
uint8_t raw_gps_log_type = 0;


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
uint8_t motors_off = 1;
uint32_t motor_off_index = 0;


// ------------------------------------------------------------------------------------------------------ Magnetometer testing

float yaw_alpha = 0.0f;

uint8_t use_compass_rpm = 1;

const uint8_t compass_frequency_values_amount = 100;

const float compass_frequency_frequency_samples[] = {
    0.000000f,
    1.747929f,
    3.495859f,
    5.243788f,
    6.991717f,
    8.739646f,
    10.487576f,
    12.235505f,
    13.983434f,
    15.731364f,
    17.479293f,
    19.227222f,
    20.975152f,
    22.723081f,
    24.471010f,
    26.218939f,
    27.966869f,
    29.714798f,
    31.462727f,
    33.210657f,
    34.958586f,
    36.706515f,
    38.454444f,
    40.202374f,
    41.950303f,
    43.698232f,
    45.446162f,
    47.194091f,
    48.942020f,
    50.689949f,
    52.437879f,
    54.185808f,
    55.933737f,
    57.681667f,
    59.429596f,
    61.177525f,
    62.925455f,
    64.673384f,
    66.421313f,
    68.169242f,
    69.917172f,
    71.665101f,
    73.413030f,
    75.160960f,
    76.908889f,
    78.656818f,
    80.404747f,
    82.152677f,
    83.900606f,
    85.648535f,
    87.396465f,
    89.144394f,
    90.892323f,
    92.640253f,
    94.388182f,
    96.136111f,
    97.884040f,
    99.631970f,
    101.379899f,
    103.127828f,
    104.875758f,
    106.623687f,
    108.371616f,
    110.119545f,
    111.867475f,
    113.615404f,
    115.363333f,
    117.111263f,
    118.859192f,
    120.607121f,
    122.355051f,
    124.102980f,
    125.850909f,
    127.598838f,
    129.346768f,
    131.094697f,
    132.842626f,
    134.590556f,
    136.338485f,
    138.086414f,
    139.834343f,
    141.582273f,
    143.330202f,
    145.078131f,
    146.826061f,
    148.573990f,
    150.321919f,
    152.069848f,
    153.817778f,
    155.565707f,
    157.313636f,
    159.061566f,
    160.809495f,
    162.557424f,
    164.305354f,
    166.053283f,
    167.801212f,
    169.549141f,
    171.297071f,
    173.045000f,
};

const float compass_frequency_mag1_x_offsets[] = {
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    -0.084852f,
    -0.157703f,
    -0.222555f,
    -0.323407f,
    -0.372259f,
    -0.352259f,
    -0.372259f,
    -0.422259f,
    -0.402259f,
    -0.464259f,
    -0.530259f,
    -0.570259f,
    -0.542259f,
    -0.560259f,
    -0.556259f,
    -0.572259f,
    -0.686259f,
    -0.774259f,
    -0.860259f,
    -0.924259f,
    -0.974259f,
    -0.952259f,
    -0.986259f,
    -1.002259f,
    -1.094259f,
    -1.126259f,
    -1.206259f,
    -1.280259f,
    -1.382259f,
    -1.470259f,
    -1.534259f,
    -1.562259f,
    -1.632259f,
    -1.744259f,
    -1.798259f,
    -1.900259f,
    -2.030259f,
    -2.078259f,
    -2.190259f,
    -2.320259f,
    -2.428259f,
    -2.520259f,
    -2.672259f,
    -2.750259f,
    -2.900259f,
    -3.062259f,
    -3.222259f,
    -3.408259f,
    -3.552259f,
    -3.668259f,
    -3.836259f,
    -4.000259f,
    -4.178259f,
    -4.354259f,
    -4.514259f,
    -4.556259f,
    -4.784259f,
    -4.992259f,
    -5.164259f,
    -5.458259f,
    -5.762259f,
    -5.958259f,
    -6.162259f,
    -6.512259f,
    -6.758259f,
    -7.052259f,
    -7.222259f,
    -7.476259f,
    -7.702259f,
    -7.824259f,
    -8.152259f,
    -8.562259f,
    -8.914259f,
    -9.278259f,
    -9.838259f,
    -10.216259f,
    -10.540259f,
    -10.890259f,
    -11.188259f,
    -11.414259f,
    -11.664259f,
    -12.194259f,
    -12.634259f,
    -12.956259f,
    -13.422259f,
    -14.002259f,
    -14.590259f,
    -15.170259f,
    -15.702259f,
    -16.044259f,
    -15.944259f,
    -16.084259f,
};

const float compass_frequency_mag1_y_offsets[] = {
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.006452f,
    0.024905f,
    0.187357f,
    0.315810f,
    0.472262f,
    0.510262f,
    0.546262f,
    0.464262f,
    0.508262f,
    0.502262f,
    0.546262f,
    0.688262f,
    0.678262f,
    0.758262f,
    0.698262f,
    0.844262f,
    0.732262f,
    0.754262f,
    0.596262f,
    0.658262f,
    0.686262f,
    0.712262f,
    0.938262f,
    1.182262f,
    1.208262f,
    1.320262f,
    1.438262f,
    1.324262f,
    1.232262f,
    1.424262f,
    1.284262f,
    1.320262f,
    1.562262f,
    1.600262f,
    1.742262f,
    1.824262f,
    1.906262f,
    1.786262f,
    1.884262f,
    1.910262f,
    1.976262f,
    2.118262f,
    2.428262f,
    2.728262f,
    2.870262f,
    3.090262f,
    3.216262f,
    3.326262f,
    3.324262f,
    3.476262f,
    3.706262f,
    3.960262f,
    4.076262f,
    4.276262f,
    4.338262f,
    4.362262f,
    4.496262f,
    4.608262f,
    4.754262f,
    4.962262f,
    5.258262f,
    5.402262f,
    5.648262f,
    5.886262f,
    6.068262f,
    6.256262f,
    6.482262f,
    6.790262f,
    7.008262f,
    7.206262f,
    7.450262f,
    7.734262f,
    7.972262f,
    8.190262f,
    8.470262f,
    8.674262f,
    8.812262f,
    8.942262f,
};

const float compass_frequency_mag1_z_offsets[] = {
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    -0.196518f,
    -0.437036f,
    -0.569553f,
    -0.756071f,
    -0.968589f,
    -0.894589f,
    -0.898589f,
    -0.836589f,
    -0.810589f,
    -0.738589f,
    -0.784589f,
    -0.836589f,
    -1.054589f,
    -1.230589f,
    -1.352589f,
    -1.592589f,
    -1.682589f,
    -1.800589f,
    -1.782589f,
    -1.796589f,
    -1.784589f,
    -1.666589f,
    -1.620589f,
    -1.612589f,
    -1.738589f,
    -1.674589f,
    -1.990589f,
    -2.330589f,
    -2.712589f,
    -3.036589f,
    -3.150589f,
    -3.088589f,
    -2.894589f,
    -2.692589f,
    -2.850589f,
    -3.058589f,
    -3.226589f,
    -3.628589f,
    -4.062589f,
    -3.924589f,
    -4.250589f,
    -4.490589f,
    -4.424589f,
    -4.584589f,
    -4.790589f,
    -5.070589f,
    -5.320589f,
    -5.772589f,
    -5.900589f,
    -6.164589f,
    -6.204589f,
    -6.322589f,
    -6.546589f,
    -6.794589f,
    -6.992589f,
    -7.332589f,
    -7.724589f,
    -7.940589f,
    -8.250589f,
    -8.796589f,
    -9.130589f,
    -9.408589f,
    -9.656589f,
    -10.154589f,
    -10.622589f,
    -11.090589f,
    -11.546589f,
    -12.156589f,
    -12.618589f,
    -13.158589f,
    -13.766589f,
    -14.418589f,
    -15.114589f,
    -15.520589f,
    -16.016589f,
    -16.180589f,
    -16.310589f,
    -16.964589f,
    -17.646589f,
    -18.044589f,
    -18.940589f,
    -20.032589f,
    -20.690589f,
    -21.176589f,
    -21.682589f,
    -21.990589f,
    -21.842589f,
    -22.092589f,
};

const float compass_frequency_mag2_x_offsets[] = {
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    -0.042466f,
    -0.062932f,
    -0.212331f,
    -0.102331f,
};

const float compass_frequency_mag2_y_offsets[] = {
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.013123f,
    0.156247f,
    0.231370f,
    0.294493f,
    0.367617f,
    0.425617f,
    0.307617f,
    0.259617f,
    0.243617f,
    0.257617f,
    0.261617f,
    0.339617f,
    0.399617f,
    0.515617f,
    0.435617f,
};

const float compass_frequency_mag2_z_offsets[] = {
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.014748f,
    0.143496f,
    0.334243f,
    0.436991f,
    0.729739f,
    0.957739f,
    1.031739f,
    1.011739f,
    1.017739f,
    0.879739f,
    0.803739f,
    0.799739f,
    0.837739f,
    0.969739f,
    0.951739f,
    0.913739f,
    0.815739f,
    0.913739f,
    0.911739f,
    0.959739f,
    1.137739f,
    1.221739f,
    1.257739f,
    1.127739f,
    1.277739f,
    1.257739f,
    1.117739f,
    1.071739f,
    1.185739f,
    1.013739f,
    0.903739f,
    1.241739f,
    1.211739f,
    1.363739f,
    1.507739f,
    1.873739f,
    1.533739f,
};

float average_rpm = 0.0f;
float mag1_x_offset = 0.0f;
float mag1_y_offset = 0.0f;
float mag1_z_offset = 0.0f;
float mag2_x_offset = 0.0f;
float mag2_y_offset = 0.0f;
float mag2_z_offset = 0.0f;

// 0 - Nothing. Let motors spin
// 1 - Calibrate accelerometer ellipsoid
// 2 - Calibrate accelerometer level
// 3 - Calibrate roll pitch offsets level
uint8_t perform_procedure = 0;

// Accelerometer stuff
uint8_t calibrate_accelerometer_step = 0;
uint8_t calibrate_accelerometer_ellipsoid_steps = 6;
uint8_t calibrate_accelerometer_level_steps = 4;

uint8_t calibrate_accelerometer_step_logged = 0;
uint16_t calibrate_accelerometer_step_logged_amount = 0;
uint16_t calibrate_accelerometer_step_logged_amount_required = 1000;

struct running_average average_accelerometer_x;
struct running_average average_accelerometer_y;
struct running_average average_accelerometer_z;

float calibrate_accelerometer_values_x[6] = {0.0f};
float calibrate_accelerometer_values_y[6] = {0.0f};
float calibrate_accelerometer_values_z[6] = {0.0f};

// Roll pitch stuff
uint8_t calibrate_roll_pitch_step = 0;
uint8_t calibrate_roll_pitch_steps = 4;

uint8_t calibrate_roll_pitch_step_logged = 0;
uint16_t calibrate_roll_pitch_step_logged_amount = 0;
uint16_t calibrate_roll_pitch_step_logged_amount_required = 1000;

struct running_average average_roll;
struct running_average average_pitch;

float calibrate_roll_pitch_values_roll[4] = {0.0f};
float calibrate_roll_pitch_values_pitch[4] = {0.0f};

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


        run_procedures();


        handle_logging();

        handle_loop_end();
    }
}


// Acc X;Acc Y;Acc Z;
// 0.015615;0.032073;-1.040398;
// 0.029494;0.516078;-0.541444;
// 0.029609;0.010337;-0.381864;
// 0.277988;0.004781;-0.300801;
// 0.028245;0.002281;-0.228635;
// 0.035407;-0.002660;-0.028402;

// Or a tab delimited .txt
// 0.015615        0.032073        -1.040398
// 0.029494        0.516078        -0.541444
// 0.029609        0.010337        -0.381864
// 0.277988        0.004781        -0.300801
// 0.028245        0.002281        -0.228635
// 0.035407        -0.002660       -0.028402



// Acc X;Acc Y;Acc Z;
// 0.016684;0.035051;-1.037436;
// 0.030078;0.517685;-0.540608;
// 0.030050;0.011670;-0.382815;
// 0.278203;0.005765;-0.303203;
// 0.028958;0.000122;-0.225134;
// 0.036209;-0.004089;-0.025103;

// Or a tab delimited .txt
// 0.016684        0.035051        -1.037436
// 0.030078        0.517685        -0.540608
// 0.030050        0.011670        -0.382815
// 0.278203        0.005765        -0.303203
// 0.028958        0.000122        -0.225134
// 0.036209        -0.004089       -0.025103

// -0.056475;0.004755;-1.033929;
// 0.043341;1.000425;-0.049028;
// 0.030660;-1.000366;-0.063235;
// 1.023309;-0.011778;-0.055051;
// -0.966882;-0.018413;0.095414;
// 0.072232;-0.025255;0.973453;



