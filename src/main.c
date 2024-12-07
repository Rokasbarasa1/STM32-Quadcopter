/* Auto generated shit -----------------------------------------------*/
#include "main.h"


I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* Actual functional code -----------------------------------------------*/

#include "../lib/printf/retarget.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include <inttypes.h>

// Drivers 
#include "../lib/mpu6050/mpu6050.h"
#include "../lib/qmc5883l/qmc5883l.h"
#include "../lib/mmc5603/mmc5603.h"

#include "../lib/bmp280/bmp280.h"
#include "../lib/bn357/bn357.h"
#include "../lib/nrf24l01/nrf24l01.h"
#include "../lib/sd_card/sd_card_spi.h"
#include "../lib/betaflight_blackbox_wrapper/betaflight_blackbox_wrapper.h"

// Other imports
#include "../lib/utils/ned_coordinates/ned_coordinates.h"
#include "../lib/pid/pid.h"
#include "../lib/filtering/filtering.h"
#include "../lib/kalman_filter/kalman_filter.h"
#include "../lib/utils/matrix_operations/matrix_operations.h"
#include "../lib/bdshot600_dma/bdshot600_dma.h"
#include "../utils/math_constants.h"
#include "../utils/reset_i2c/reset_i2c.h"

void init_STM32_peripherals();
void calibrate_escs();
uint16_t setServoActivationPercent(float percent, uint16_t minValue, uint16_t maxValue);
float mapValue(float value, float input_min, float input_max, float output_min, float output_max);
void extract_request_values(char *request, uint8_t request_size, uint8_t *throttle, uint8_t *yaw, uint8_t *pitch, uint8_t *roll);
float get_sensor_fusion_altitude(float gps_altitude, float barometer_altitude);
u_int8_t init_drivers();
void check_calibrations();
void calibrate_gyro();
void get_initial_position();
void handle_loop_timing();
void reset_array_data(float *array, uint8_t array_size);

void extract_joystick_request_values_uint(char *request, uint8_t request_size, uint8_t *throttle, uint8_t *yaw, uint8_t *roll, uint8_t *pitch);
void extract_joystick_request_values_float(char *request, uint8_t request_size, float *throttle, float *yaw, float *roll, float *pitch);
void extract_request_type(char *request, uint8_t request_size, char *type_output);
void extract_pid_request_values(char *request, uint8_t request_size, float *added_proportional, float *added_integral, float *added_derivative, float *added_master);
void extract_accelerometer_offsets(char *request, uint8_t request_size, float *added_x_axis_offset, float *added_y_axis_offset);
float map_value(float value, float input_min, float input_max, float output_min, float output_max);
float limit_max_value(float value, float min_value, float max_value);
float apply_dead_zone(float value, float max_value, float min_value, float dead_zone);
void send_pid_base_info_to_remote();
void send_pid_added_info_to_remote();
char* generate_message_pid_values_nrf24(float base_proportional, float base_integral, float base_derivative, float base_master);
void extract_flight_mode(char *request, uint8_t request_size, uint8_t *new_flight_mode);
void switch_x_and_y_axis(float *data);
void invert_axies(float *data);
float sawtooth_sin(float x_radian);
float sawtooth_cos(float x_radian);
float triangle_wave(float x);
float triangle_sin(float x);
float triangle_cos(float x);
float map_value_to_expo_range(float value, float min_expo, float max_expo, uint8_t expo_curve);
float map_value_to_expo_range_inverted(float value, float min_expo, float max_expo, uint8_t expo_curve);

void handle_get_and_calculate_sensor_values();
void handle_radio_communication();
void handle_logging();
void handle_pid_and_motor_control();
void setup_logging_to_sd(uint8_t use_updated_file_name);
void handle_pre_loop_start();
void handle_loop_start();
void handle_loop_end();
uint64_t get_absolute_time();
void initialize_control_abstractions();
void set_flight_mode(uint8_t mode);
void initialize_motor_communication();

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
// Nyquist: sampling frequency must be at least double that of the highest frequency of interest
// The MPU6050 has a lowpass set to 260Hz we need to be able to manipulate all the frequencies until that. So 260 * 2 = 520
#define REFRESH_RATE_HZ 521

const uint16_t precalculated_timing_miliseconds = (1000000 / REFRESH_RATE_HZ);
// ------------------------------------------------------------------------------------------------------ handling loop timing
// Timing
// TODO: Change this variable to uint64 later
uint64_t absolute_microseconds_since_start = 0;
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

#define max_dshot600_throttle_value 2047
#define min_dshot600_throttle_value 48

#define max_dshot600_command_value 47
#define min_dshot600_command_value 1

#define dshot600_neutral_value 0
#define actual_min_dshot600_throttle_value 91 // Lowest value that lets the motors spin freely and at low rpm
// Take 75 percent of max because my battery can't handle all that current.
const uint16_t actual_max_dshot600_throttle_value = min_dshot600_throttle_value + ((max_dshot600_throttle_value - min_dshot600_throttle_value) * 75) / 100; // (max lipo amp rating / max draw of a bldc motor being used x 4) 0.917 * (max_pwm - min_pwm) + min_pwm = 371.4

// Motor 0 - BL, 1 - BR, 2 - FR, 3 - FL
float motor_power[] = {0.0, 0.0, 0.0, 0.0}; // Percent
float motor_frequency[] = {0.0, 0.0, 0.0, 0.0};
float motor_rpm[] = {0.0, 0.0, 0.0, 0.0};
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

// ------------------------------------------------------------------------------------------------------ Sensor calibrations

// Convert readings from this site to same units nanoTeslas to microteslas
// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
// When calculating this remember that these values are already
// correcting and that the axis of the magnetometer are switched.

// For calibrating the magnetometer I
// used a method found in MicWro Engr Video:
// https://www.youtube.com/watch?v=MinV5V1ioWg
// Mag field norm at my location is 50.503
// use software called Magneto 1.2

// motor 2807

// Magnetometer
float magnetometer_hard_iron_correction[3] = {
    3274.370177,3280.305541,3275.211456
};

float magnetometer_soft_iron_correction[3][3] = {
    {1.021759,0.011752,0.066446},
    {0.011752,1.121668,-0.066759},
    {0.066446,-0.066759,1.172338}
};

// Accelerometer
float accelerometer_correction[3] = {
    0.048688, -0.011751, 1.118072
};

float accelerometer_scale_factor_correction[3][3] = {
    {0.989138,-0.0125890,0.004309},
    {-0.012589,0.997708,-0.001005},
    {0.004309,-0.001005,0.972671}
};

// Gyro
float gyro_correction[3] = {
    -2.532809, 2.282250, 0.640916
};

float base_accelerometer_roll_offset = -1.74464;
float base_accelerometer_pitch_offset = 3.11469;

float accelerometer_roll_offset = 0;
float accelerometer_pitch_offset = 0;

float yaw_offset = 2.77420594;

// ------------------------------------------------------------------------------------------------------ Accelerometer values to degrees conversion
float accelerometer_roll = 0;
float accelerometer_pitch = 0;
float magnetometer_yaw = 0;

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

struct pid vertical_velocity_pid;

struct pid gps_longitude_pid;
struct pid gps_latitude_pid;

float error_acro_roll = 0;
float error_acro_pitch = 0;
float error_acro_yaw = 0;

float error_angle_pitch = 0;
float error_angle_roll = 0;

float error_throttle = 0;
float error_vertical_velocity = 0;

float error_latitude = 0;
float error_longitude = 0;
float last_error_roll = 0;
float last_error_pitch = 0;


// Notes for PID
// 1) It is important that the natural state without power of the drone is not stable, otherwise 
// some offset of center off mass or pid desired point needs to introduced
// 2) Remember that the drone when in the air has motors spinning at idle power, just enough 
// to float in the air. If you are testing with drone constrained add a base motor speed to account for this.


// Actual PID adjustment for pitch
#define BASE_ANGLE_roll_pitch_MASTER_GAIN 1.0
#define BASE_ANGLE_roll_pitch_GAIN_P 2.0
#define BASE_ANGLE_roll_pitch_GAIN_I 0.0
#define BASE_ANGLE_roll_pitch_GAIN_D 0.0

#define BASE_ACRO_roll_pitch_MASTER_GAIN 0.4
#define BASE_ACRO_roll_pitch_GAIN_P 1.2 // 0.7
#define BASE_ACRO_roll_pitch_GAIN_I 0.0 // 0.0
#define BASE_ACRO_roll_pitch_GAIN_D 0.02 // 0.01

// Used for smooth changes to PID while using remote control. Do not touch this
float angle_roll_pitch_master_gain = BASE_ANGLE_roll_pitch_MASTER_GAIN;
float angle_roll_pitch_gain_p = BASE_ANGLE_roll_pitch_GAIN_P;
float angle_roll_pitch_gain_i = BASE_ANGLE_roll_pitch_GAIN_I;
float angle_roll_pitch_gain_d = BASE_ANGLE_roll_pitch_GAIN_D;

float added_angle_roll_pitch_master_gain = 0;
float added_angle_roll_pitch_gain_p = 0;
float added_angle_roll_pitch_gain_i = 0;
float added_angle_roll_pitch_gain_d = 0;

float acro_roll_pitch_master_gain = BASE_ACRO_roll_pitch_MASTER_GAIN;
float acro_roll_pitch_gain_p = BASE_ACRO_roll_pitch_GAIN_P;
float acro_roll_pitch_gain_i = BASE_ACRO_roll_pitch_GAIN_I;
float acro_roll_pitch_gain_d = BASE_ACRO_roll_pitch_GAIN_D;

float added_acro_roll_pitch_master_gain = 0;
float added_acro_roll_pitch_gain_p = 0;
float added_acro_roll_pitch_gain_i = 0;
float added_acro_roll_pitch_gain_d = 0;

// PID for acro yaw
const float acro_yaw_gain_p = 0.45;
const float acro_yaw_gain_i = 0.3;

// PID for altitude control
const float vertical_velocity_gain_p = 1.2; 
const float vertical_velocity_gain_i = 3.50;
const float vertical_velocity_gain_d = 0.1;

// PID for gps hold
const float gps_hold_gain_p = 700000.0; 
const float gps_hold_gain_i = 0.0;
const float gps_hold_gain_d = 21000.0;

float acro_target_roll = 0.0;
float acro_target_pitch = 0.0;
float acro_target_yaw = 0.0;

float angle_target_pitch = 0.0;
float angle_target_roll = 0.0;

float target_vertical_velocity = 0.0;

float target_altitude = 0.0;
float target_longitude = 0.0;
float target_latitude = 0.0;

// ------------------------------------------------------------------------------------------------------ Some configurations relating to PID
const float gps_pid_angle_of_attack_max = 5.0;

float acro_mode_rate_degrees_per_second = 100;

// ------------------------------------------------------------------------------------------------------ Sensor other data
float acceleration_data[] = {0,0,0};
float gyro_angular[] = {0,0,0};
float imu_orientation[] = {0,0,0};
float magnetometer_data[] = {0,0,0};
float gps_longitude = 0.0;
float old_gps_longitude = 0.0;
float gps_latitude = 0.0;
float old_gps_latitude = 0.0;
float delta_gps_latitude = 0.0;
float delta_gps_longitude = 0.0;
float in_between_gps_latitude = 0.0;
float in_between_gps_longitude = 0.0;
uint8_t in_between_gps_loop_divider = 50;
uint8_t in_between_gps_loop_divider_counter = 0;
uint8_t gps_fix_type = 0.0;
uint8_t got_gps = 0;

float pressure = 0.0;
float temperature = 0.0;
float altitude = 0.0;
float vertical_velocity = 0.0;


uint8_t gps_position_hold_enabled = 0;
uint8_t gps_target_set = 0;

uint8_t use_gps_reset_count = 0;
uint8_t use_gps_reset_count_to_deactivate = 10; // This is done because sometimes some noise sends a neutural pitch and roll

float gps_hold_roll_adjustment_integral = 0.0;
float gps_hold_pitch_adjustment_integral = 0.0;

float roll_effect_on_lat = 0;
float pitch_effect_on_lat = 0;

float roll_effect_on_lon = 0;
float pitch_effect_on_lon = 0;

// ------------------------------------------------------------------------------------------------------ Sensor fusion and filtering stuff
float complementary_beta = 0.0;
float complementary_ratio = 1.0 - 1.0/(1.0+(1.0/REFRESH_RATE_HZ)); // Depends on how often the loop runs. 1 second / (1 second + one loop time)

// This is used for filtering limits
float nyquist_with_margin = REFRESH_RATE_HZ / 2.0f - 6.0f; // The -6 is the margin that greatly improves the quality of filtering for notch filters

float motor_low_pass_filter_cutoff = 150;
struct low_pass_pt1_filter filter_motor_0;
struct low_pass_pt1_filter filter_motor_1;
struct low_pass_pt1_filter filter_motor_2;
struct low_pass_pt1_filter filter_motor_3;

float filtering_magnetometer_cutoff_frequency = 180;
struct low_pass_pt1_filter filter_magnetometer_x;
struct low_pass_pt1_filter filter_magnetometer_y;
struct low_pass_pt1_filter filter_magnetometer_z;

float filtering_accelerometer_cutoff_frequency = 15;
struct low_pass_biquad_filter biquad_filter_accelerometer_x;
struct low_pass_biquad_filter biquad_filter_accelerometer_y;
struct low_pass_biquad_filter biquad_filter_accelerometer_z;

struct low_pass_pt1_filter filter_gyro_z;

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

struct low_pass_biquad_filter roll_d_term_filtering;
struct low_pass_biquad_filter pitch_d_term_filtering;

float gyro_angular_for_d_term[] = {0,0,0};

const float d_term_filtering_min_cutoff = 75;
const float d_term_filtering_max_cutoff = 100;
const uint8_t d_term_filtering_expo = 7;

struct kalman_filter altitude_and_velocity_kalman;
// ------------------------------------------------------------------------------------------------------ Remote control settings
float max_yaw_attack = 40.0;
float max_roll_attack = 25;
float max_pitch_attack = 25;
float roll_attack_step = 0.2;
float pitch_attack_step = 0.2;
float max_throttle_vertical_velocity = 30; // cm/second

float max_angle_before_motors_off = 40;

float throttle = 0.0;
float yaw = 50.0;
float last_yaw = 50.0;
float pitch = 50.0;
float roll = 50.0;

float minimum_signal_timing_seconds = 0.2; // Seconds
uint64_t last_signal_timestamp_microseconds = 0;

// ------------------------------------------------------------------------------------------------------ Logging to SD card
char log_file_base_name[] = "Quadcopter.txt";
char log_file_blackbox_base_name[] = "Quadcopter.BBL";
char *new_log_file_blackbox_base_name;
uint16_t file_index = 0;

const uint8_t write_repeat_logs_in_same_file = 1;

uint8_t sd_card_initialized = 0;
uint8_t log_loop_count = 0;
uint8_t sd_card_async = 0;
const uint8_t use_simple_async = 0; // 0 is the complex async and is faster
const uint8_t use_blackbox_logging = 1; 


void DMA1_Stream7_IRQHandler(void){
    HAL_DMA_IRQHandler(&hdma_spi3_tx);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    if (hspi->Instance == SPI3){
        sd_card_set_dma_transfer_call_status(0); // Set to zero as dma is not happening anymore
    }
}

// ------------------------------------------------------------------------------------------------------ Stuff for blackbox logging
uint32_t loop_iteration = 1;
float PID_proportional[3];
float PID_integral[3];
float PID_derivative[2];
float PID_feed_forward[3];
float PID_set_points[4];
float remote_control[4];

// ------------------------------------------------------------------------------------------------------ Flight mode settings
// Flight modes:
// (0) - acro
// (1) - angle
// (2) - angle with altitude hold
// (3) - angle with altitude hold and gps hold
// (4) - angle with gps hold and without altitude hold

uint8_t flight_mode = 1;

uint8_t use_angle_mode = 0;
uint8_t use_vertical_velocity_control = 0;
uint8_t use_gps_hold = 0;

int main(void){
    init_STM32_peripherals();
    accelerometer_roll_offset = base_accelerometer_roll_offset;
    accelerometer_pitch_offset = base_accelerometer_pitch_offset;
    // Turn off the blue led. Will show the status of sd logging later
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    printf("STARTING PROGRAM\n"); 
    

    
    if(init_drivers() == 0) return 0; // exit if initialization failed

    HAL_Delay(2000); // Dont want to interfere in calibration

    set_flight_mode(flight_mode);
    // check_calibrations();
    calibrate_gyro(); // Recalibrate the gyro as the temperature affects the calibration
    get_initial_position();
    setup_logging_to_sd(0); // If stopped working then re-flash the firmware of the logger and check hardware connection. Check if it works when Li-po connected
    initialize_control_abstractions();
    initialize_motor_communication();

    handle_pre_loop_start();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  
    DWT->CYCCNT = 0;                                // Reset the cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    while (1){
        // Get current time
        handle_loop_start();
        DWT->CYCCNT = 0;

        radio = DWT->CYCCNT;
        handle_radio_communication();
        radio_end = DWT->CYCCNT;

        sensors = DWT->CYCCNT;
        handle_get_and_calculate_sensor_values(); // Important do do this right before the pid stuff.
        sensors_end = DWT->CYCCNT;

        motor = DWT->CYCCNT;
        handle_pid_and_motor_control();
        motor_end = DWT->CYCCNT;

        logging = DWT->CYCCNT;
        handle_logging();
        logging_end = DWT->CYCCNT;

        handle_loop_end();

        // printf("ACCEL,%.2f,%.2f,%.2f;", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
        // printf("GYRO,%.2f,%.2f,%.2f;", gyro_angular[0], gyro_angular[1], gyro_angular[2]);
        // printf("MAG,%.2f,%.2f,%.2f;", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);
        // if(loop_iteration % 20 == 0){
        //     printf("IMU,%.2f,%.2f,%.2f;", imu_orientation[0], imu_orientation[1], imu_orientation[2]);
        //     printf("\n");
        // }
        
        // printf("2r %lu\n", radio2 - radio);
        // printf("r %lu\n", radio_end - radio);

        // printf("2s %lu\n", sensors2 - sensors);
        // printf("3s %lu\n", sensors3 - sensors2);
        // printf("4s %lu\n", sensors4 - sensors3);
        // printf("5s %lu\n", sensors5 - sensors4);
        // printf("6s %lu\n", sensors6 - sensors5);
        // printf("7s %lu\n", sensors7 - sensors6);
        // printf("8s %lu\n", sensors8 - sensors7);
        // printf("9s %lu\n", sensors9 - sensors8);
        // printf("10s %lu\n", sensors10 - sensors9);
        // printf("11s %lu\n", sensors11 - sensors10);
        // printf("s %lu\n", sensors_end - sensors);

        // printf("2m %lu\n", motor2 - motor);
        // printf("3m %lu\n", motor3 - motor2);
        // printf("4m %lu\n", motor4 - motor3);
        // printf("5m %lu\n", motor5 - motor4);
        // printf("6m %lu\n", motor6 - motor6);
        // printf("7m %lu\n", motor7 - motor6);
        // printf("m %lu\n", motor_end - motor);

        // printf("2L %lu\n", logging2 - logging);
        // printf("3L %lu\n", logging3 - logging2);
        // printf("L %lu\n", logging_end - logging);

        // printf("IN %lu\n", interrupt_end - interrupt);

        // printf("cv_all %lu\n", bdshot600_dma_get_timings()[4] - bdshot600_dma_get_timings()[0]);
        // printf("cv_all2 %lu\n", bdshot600_dma_get_timings()[2] - bdshot600_dma_get_timings()[0]);
        // printf("cv_all3 %lu\n", bdshot600_dma_get_timings()[3] - bdshot600_dma_get_timings()[2]);
        // printf("cv_all5 %lu\n", bdshot600_dma_get_timings()[4] - bdshot600_dma_get_timings()[3]);

        // printf("pre_res %lu\n", bdshot600_dma_get_timings()[6] - bdshot600_dma_get_timings()[5]);

        // printf("cv_res %lu\n", bdshot600_dma_get_timings()[14] - bdshot600_dma_get_timings()[7]);
        // printf("cv_res8 %lu\n", bdshot600_dma_get_timings()[8] - bdshot600_dma_get_timings()[7]);
        // printf("cv_res9 %lu\n", bdshot600_dma_get_timings()[9] - bdshot600_dma_get_timings()[8]);
        // printf("cv_res10 %lu\n", bdshot600_dma_get_timings()[10] - bdshot600_dma_get_timings()[9]);
        // printf("cv_res11 %lu\n", bdshot600_dma_get_timings()[11] - bdshot600_dma_get_timings()[10]);
        // printf("cv_res12 %lu\n", bdshot600_dma_get_timings()[12] - bdshot600_dma_get_timings()[11]);
        // printf("cv_res13 %lu\n", bdshot600_dma_get_timings()[13] - bdshot600_dma_get_timings()[12]);
        // printf("cv_res14 %lu\n", bdshot600_dma_get_timings()[14] - bdshot600_dma_get_timings()[13]);

        // printf("cv_resin15 %lu\n", bdshot600_dma_get_timings()[15] - bdshot600_dma_get_timings()[8]);
        // printf("cv_resin16 %lu\n", bdshot600_dma_get_timings()[16] - bdshot600_dma_get_timings()[15]);
        // printf("cv_resin17 %lu\n", bdshot600_dma_get_timings()[17] - bdshot600_dma_get_timings()[16]);

        // printf("(%lu + %lu + %lu + %lu + %lu)/100000000 = %.5f\n\n", 
        //     radio_end - radio,
        //     sensors_end - sensors,
        //     motor_end - motor,
        //     logging_end - logging,
        //     interrupt_end - interrupt,
        //     ((float)(radio_end - radio) + (sensors_end - sensors) + (motor_end - motor) + (logging_end - logging) + (interrupt_end - interrupt)) / 100000000.0
        // );

        // printf("Motor BL: %5.1f BR: %5.1f FR: %5.1f FL: %5.1f\n",
        //     motor_rpm[0],
        //     motor_rpm[1],
        //     motor_rpm[2],
        //     motor_rpm[3]
        // );

        // printf("%.1f\n",
        //     motor_rpm[0]
        // );


    }
}

uint8_t init_drivers(){
    printf("-----------------------------INITIALIZING MODULES...\n");

    uint8_t mpu6050 = init_mpu6050(
        &hi2c1, 
        ACCEL_CONFIG_RANGE_2G, 
        GYRO_CONFIG_RANGE_500_DEG, 
        LOW_PASS_FILTER_FREQUENCY_CUTOFF_GYRO_260HZ_ACCEL_256HZ,
        1, 
        accelerometer_scale_factor_correction, 
        accelerometer_correction, 
        gyro_correction, 
        REFRESH_RATE_HZ, 
        complementary_ratio, 
        complementary_beta
    );

    uint8_t mmc5603 = mmc5603_init(&hi2c1, 1, magnetometer_hard_iron_correction, magnetometer_soft_iron_correction, 1, 200, 1);
    uint8_t bmp280 = init_bmp280(&hi2c1);
    uint8_t bn357 = init_bn357(&huart2, &hdma_usart2_rx, 1);
    uint8_t nrf24 = init_nrf24(&hspi1);

    printf("-----------------------------INITIALIZING MODULES DONE... ");

    if (mpu6050 && mmc5603 && bmp280 && bn357 && nrf24){
        printf("OK\n");
    }else{
        printf("NOT OK\n");
        return 0;
    }

    // Continue initializing
    nrf24_rx_mode(tx_address, 10);
    bn357_start_uart_interrupt();

    return 1;
}

void handle_get_and_calculate_sensor_values(){
    // ------------------------------------------------------------------------------------------------------ Reset
    // Reset the values to make it easier to find broken sensor later
    reset_array_data(acceleration_data, 3);
    reset_array_data(gyro_angular, 3);
    reset_array_data(magnetometer_data, 3);
    temperature = 0.0f;
    altitude = 0.0f;
    vertical_velocity = 0.0f;

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
    // temperature = bmp280_get_temperature_celsius(); // Temperature will probably not change
    altitude = bmp280_get_height_centimeters_from_reference(0);
    mmc5603_magnetometer_readings_micro_teslas(magnetometer_data); // Call other sensor to let mpu6050 to rest
    sensors3 = DWT->CYCCNT;
    mpu6050_get_accelerometer_readings_gravity(acceleration_data);
    sensors4 = DWT->CYCCNT;
    mpu6050_get_gyro_readings_dps(gyro_angular);
    __enable_irq();
    sensors5 = DWT->CYCCNT;


    bn357_parse_data(); // Try to parse gps
    gps_latitude = bn357_get_latitude_decimal_format();
    gps_longitude = bn357_get_linear_longitude_decimal_format(); // Linear for pid
    gps_fix_type = bn357_get_fix_type();
    got_gps = bn357_get_status_up_to_date(1);

    // Pitch (+)forwards - front of device nose goes down
    // Roll (+)right - the device banks to the right side while pointing forward
    // After yaw calculation yaw has to be 0/360 when facing north in the pitch+ axis
    // After getting roll and pitch from accelerometer. Pitch forwards -> +Pitch. Roll right -> +Roll
    rotate_magnetometer_output_90_degrees_anti_clockwise(magnetometer_data);
    invert_axies(acceleration_data);
    invert_axies(gyro_angular);
    // Pitch device forwards -> Y axis positive. Roll device right -> X axis positive

    // ------------------------------------------------------------------------------------------------------ Filter the sensor data
    sensors6 = DWT->CYCCNT;

    // Notch filter: https://www.youtube.com/watch?v=ysS4bIXFAsU
    // Filtering rpm: https://www.youtube.com/watch?v=NCDyA9bnf4I&t=192s


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


    // Filter magnetometer data
    magnetometer_data[0] = low_pass_filter_pt1_read(&filter_magnetometer_x, magnetometer_data[0]);
    magnetometer_data[1] = low_pass_filter_pt1_read(&filter_magnetometer_y, magnetometer_data[1]);
    magnetometer_data[2] = low_pass_filter_pt1_read(&filter_magnetometer_z, magnetometer_data[2]);

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

    // ------------------------------------------------------------------------------------------------------ Sensor fusion
    calculate_roll_pitch_from_accelerometer_data(acceleration_data, &accelerometer_roll, &accelerometer_pitch, accelerometer_roll_offset, accelerometer_pitch_offset); // Get roll and pitch from the data. I call it x and y. Ranges -90 to 90. 
    sensor_fusion_roll_pitch(gyro_angular, accelerometer_roll, accelerometer_pitch, get_absolute_time(), 1, imu_orientation);

    // Sensor fusion imu degrees > accelerometer degrees
    calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data, &magnetometer_yaw, imu_orientation[0], imu_orientation[1], yaw_offset);

    // No need for sensor fusion, it introduces delay
    imu_orientation[2] = magnetometer_yaw;
    
    sensors9 = DWT->CYCCNT;
    // GPS STUFF
    float latitude_sign = 1 * (bn357_get_latitude_direction() == 'N') + (-1) * (bn357_get_latitude_direction() == 'S'); // Possible results: 0, 1 or -1
    float longitude_sign = 1 * (bn357_get_longitude_direction() == 'E') + (-1) * (bn357_get_longitude_direction() == 'W');
    roll_effect_on_lat = -triangle_cos(imu_orientation[0] * M_PI_DIV_BY_180) * latitude_sign; // + GOOD, - GOOD  // If moving north roll right positive, roll left south negative
    pitch_effect_on_lat = triangle_cos(imu_orientation[1] * M_PI_DIV_BY_180) * latitude_sign; // + GOOD, - GOOD  // If moving north forward positive, backwards negative
    roll_effect_on_lon = triangle_cos(imu_orientation[0] * M_PI_DIV_BY_180) * longitude_sign; // GOOD, - GOOD
    pitch_effect_on_lon = triangle_cos(imu_orientation[1] * M_PI_DIV_BY_180) * longitude_sign; // + GOOD, - GOOD 

    // printf("PLAT: %.2f RLAT: %.2f PLON: %.2f RLON: %.2f\n", 
    //     pitch_effect_on_lat,
    //     roll_effect_on_lat,
    //     pitch_effect_on_lon,
    //     roll_effect_on_lon,
    // );

    // Calculate the in between latitude and longitude to help derivative value
    // if(got_gps){
    //     // Calculate the rate of change of gps at it current refresh rate
    //     delta_gps_latitude = gps_latitude - old_gps_latitude;
    //     delta_gps_longitude = gps_longitude - old_gps_longitude;

    //     // Save the current gps values as old
    //     old_gps_latitude = gps_latitude;
    //     old_gps_longitude = gps_longitude;

    //     in_between_gps_latitude = gps_latitude;
    //     in_between_gps_longitude = gps_longitude;
    //     in_between_gps_loop_divider_counter = 0;
    // }else{
        
    //     // Sometimes the update is skipped on purpose
    //     if(in_between_gps_loop_divider_counter == in_between_gps_loop_divider){
    //         // Update the inbetween
    //         in_between_gps_latitude += delta_gps_latitude * (1.0f/REFRESH_RATE_HZ) * 10;
    //         in_between_gps_longitude += delta_gps_longitude * (1.0f/REFRESH_RATE_HZ) * 10; // Temp times 10 to signify the 10 hz refresh rate of gps

    //         in_between_gps_loop_divider_counter = 0;
    //     }else{
    //         in_between_gps_loop_divider_counter++;
    //     }

    //     // Apply the inbetween 
    //     gps_latitude = in_between_gps_latitude;
    //     gps_longitude = in_between_gps_longitude;
    // }

    sensors10 = DWT->CYCCNT;
    // ------------------------------------------------------------------------------------------------------ Use sensor fused data for more data
    float vertical_acceleration[1] = {mpu6050_calculate_vertical_acceleration_cm_per_second(acceleration_data, imu_orientation)};
    float vertical_altitude[1] = {altitude};

    kalman_filter_predict(&altitude_and_velocity_kalman, vertical_acceleration);
    kalman_filter_update(&altitude_and_velocity_kalman, vertical_altitude);

    altitude = kalman_filter_get_state(&altitude_and_velocity_kalman)[0][0];
    vertical_velocity = kalman_filter_get_state(&altitude_and_velocity_kalman)[1][0];
    sensors11 = DWT->CYCCNT;

    // printf("Alt %f vel %f ", altitude, vertical_velocity);
    // printf("IMU %f %f %f\n", imu_orientation[0], imu_orientation[1], imu_orientation[2]);
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


        pid_reset_integral_sum(&acro_roll_pid);
        pid_reset_integral_sum(&acro_pitch_pid);
        pid_reset_integral_sum(&acro_yaw_pid);

        pid_reset_integral_sum(&angle_pitch_pid);
        pid_reset_integral_sum(&angle_roll_pid);

        pid_reset_integral_sum(&gps_longitude_pid);
        pid_reset_integral_sum(&gps_latitude_pid);

        // Immediately after getting the motor values send them to motors. If manual mode is on
        if(manual_bdshot) bdshot600_dma_send_all_motor_data();
        motor7 = DWT->CYCCNT;
        return;
    }
    // printf("GO   %.1f %.1f - %f\n", imu_orientation[0], imu_orientation[1], ((float)get_absolute_time() - (float)last_signal_timestamp_microseconds) / 1000000.0);

    // ---------------------------------------------------------------------------------------------- GPS position hold
    float gps_hold_roll_adjustment = 0.0f;
    float gps_hold_pitch_adjustment = 0.0f;
    if(use_gps_hold){
        if(gps_position_hold_enabled && got_gps){
            pid_set_desired_value(&gps_latitude_pid, target_latitude);
            pid_set_desired_value(&gps_longitude_pid, target_longitude);

            error_latitude = pid_get_error(&gps_latitude_pid, gps_latitude, get_absolute_time());
            error_longitude = pid_get_error(&gps_longitude_pid, gps_longitude, get_absolute_time());


            float gps_hold_roll_adjustment_calculation = roll_effect_on_lat * error_latitude + roll_effect_on_lon * error_longitude;
            float gps_hold_pitch_adjustment_calculation = pitch_effect_on_lat * error_latitude + pitch_effect_on_lon * error_longitude;


            // if(gps_hold_roll_adjustment_calculation > gps_pid_angle_of_attack_max){
            //     gps_hold_roll_adjustment_calculation = gps_pid_angle_of_attack_max;
            // }else if(gps_hold_roll_adjustment_calculation < -gps_pid_angle_of_attack_max){
            //     gps_hold_roll_adjustment_calculation = -gps_pid_angle_of_attack_max;
            // } 
        
            // if(gps_hold_pitch_adjustment_calculation > gps_pid_angle_of_attack_max){
            //     gps_hold_pitch_adjustment_calculation = gps_pid_angle_of_attack_max;
            // }else if(gps_hold_pitch_adjustment_calculation < -gps_pid_angle_of_attack_max){
            //     gps_hold_pitch_adjustment_calculation = -gps_pid_angle_of_attack_max;
            // }
            
            // gps_hold_roll_adjustment = gps_hold_roll_adjustment_integral + map_value(
            //     gps_hold_roll_adjustment_calculation, 
            //     -gps_pid_angle_of_attack_max, 
            //     gps_pid_angle_of_attack_max, 
            //     -pitch_attack_step, 
            //     pitch_attack_step
            // );

            // gps_hold_pitch_adjustment = gps_hold_pitch_adjustment_calculation + map_value(
            //     gps_hold_pitch_adjustment_calculation, 
            //     -gps_pid_angle_of_attack_max, 
            //     gps_pid_angle_of_attack_max, 
            //     -pitch_attack_step, 
            //     pitch_attack_step
            // );

            // if(gps_hold_roll_adjustment > gps_pid_angle_of_attack_max){
            //     gps_hold_roll_adjustment = gps_pid_angle_of_attack_max;
            // }else if(gps_hold_roll_adjustment < -gps_pid_angle_of_attack_max){
            //     gps_hold_roll_adjustment = -gps_pid_angle_of_attack_max;
            // }
            
            // if(gps_hold_pitch_adjustment > gps_pid_angle_of_attack_max){
            //     gps_hold_pitch_adjustment = gps_pid_angle_of_attack_max;
            // }else if(gps_hold_pitch_adjustment < -gps_pid_angle_of_attack_max){
            //     gps_hold_pitch_adjustment = -gps_pid_angle_of_attack_max;
            // }



            if(gps_hold_roll_adjustment_calculation > gps_pid_angle_of_attack_max){
                gps_hold_roll_adjustment_calculation = gps_pid_angle_of_attack_max;
            }else if(gps_hold_roll_adjustment_calculation < -gps_pid_angle_of_attack_max){
                gps_hold_roll_adjustment_calculation = -gps_pid_angle_of_attack_max;
            } 
        
            if(gps_hold_pitch_adjustment_calculation > gps_pid_angle_of_attack_max){
                gps_hold_pitch_adjustment_calculation = gps_pid_angle_of_attack_max;
            }else if(gps_hold_pitch_adjustment_calculation < -gps_pid_angle_of_attack_max){
                gps_hold_pitch_adjustment_calculation = -gps_pid_angle_of_attack_max;
            }

            gps_hold_roll_adjustment = gps_hold_roll_adjustment_calculation;
            gps_hold_pitch_adjustment = gps_hold_pitch_adjustment_calculation;
            
            // printf("%f;%f;%f;%f;%.2f;%.2f;%.2f;%.2f;%.2f;", target_latitude, target_longitude, gps_latitude, gps_longitude, gps_hold_roll_adjustment, gps_hold_pitch_adjustment, imu_orientation[0], imu_orientation[1], imu_orientation[2]);
            // printf("%.2f;%.2f;%.2f;%.2f;", roll_effect_on_lat, roll_effect_on_lon, pitch_effect_on_lat, pitch_effect_on_lon);
            // printf("\n");

            last_error_roll = gps_hold_roll_adjustment;
            last_error_pitch = gps_hold_pitch_adjustment;

            // printf("off: %3.1f %3.1f\n", gps_hold_roll_adjustment, gps_hold_pitch_adjustment);
        }else if(gps_position_hold_enabled && got_gps == 0){
            // Put in the same values'
            gps_hold_roll_adjustment = last_error_roll;
            gps_hold_pitch_adjustment = last_error_pitch;
        }
    }
    motor2 = DWT->CYCCNT;
    

    // ---------------------------------------------------------------------------------------------- Altitude hold
    if(use_vertical_velocity_control){
        pid_set_desired_value(&vertical_velocity_pid, target_vertical_velocity);
        error_vertical_velocity = pid_get_error(&vertical_velocity_pid, vertical_velocity, get_absolute_time());
    }else{
        error_vertical_velocity = 0;
    }

    motor3 = DWT->CYCCNT;

    // ---------------------------------------------------------------------------------------------- Angle mode
    if(use_angle_mode){
        pid_set_desired_value(&angle_roll_pid, angle_target_roll + gps_hold_roll_adjustment);
        pid_set_desired_value(&angle_pitch_pid, angle_target_pitch + gps_hold_pitch_adjustment);

        // For now not logging desired angle mode
        error_angle_roll = limit_max_value(pid_get_error(&angle_roll_pid, imu_orientation[0], get_absolute_time()), -acro_mode_rate_degrees_per_second, acro_mode_rate_degrees_per_second);
        error_angle_pitch = limit_max_value(pid_get_error(&angle_pitch_pid, imu_orientation[1], get_absolute_time()), -acro_mode_rate_degrees_per_second, acro_mode_rate_degrees_per_second);

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
    uint64_t pid_time = get_absolute_time();
    pid_calculate_error_pi(&acro_roll_pid, gyro_angular[0], pid_time); // Calculate P and I terms
    pid_calculate_error_d(&acro_roll_pid, gyro_angular_for_d_term[0], pid_time); // Calculate D term using a low pass filtered value
    pid_set_previous_time(&acro_roll_pid, pid_time); // Update last timestamp

    pid_calculate_error_pi(&acro_pitch_pid, gyro_angular[1], pid_time);
    pid_calculate_error_d(&acro_pitch_pid, gyro_angular_for_d_term[1], pid_time);
    pid_set_previous_time(&acro_pitch_pid, pid_time);

    pid_calculate_error(&acro_yaw_pid, gyro_angular[2], pid_time);
    pid_set_previous_time(&acro_yaw_pid, pid_time);

    PID_proportional[0] = pid_get_last_proportional_error(&acro_roll_pid);
    PID_integral[0] = pid_get_last_integral_error(&acro_roll_pid);
    PID_derivative[0] = pid_get_last_derivative_error(&acro_roll_pid);
    PID_proportional[1] = pid_get_last_proportional_error(&acro_pitch_pid);
    PID_integral[1] = pid_get_last_integral_error(&acro_pitch_pid);
    PID_derivative[1] = pid_get_last_derivative_error(&acro_pitch_pid);
    PID_proportional[2] = pid_get_last_proportional_error(&acro_yaw_pid);
    PID_integral[2] = pid_get_last_integral_error(&acro_yaw_pid);

    // Use throttle to make the biquad filter dynamic
    error_acro_roll = PID_proportional[0] + PID_integral[0] + PID_derivative[0];
    error_acro_pitch = PID_proportional[1] + PID_integral[1] + PID_derivative[1];
    error_acro_yaw = PID_proportional[2] + PID_integral[2];

    motor5 = DWT->CYCCNT;

    // ---------------------------------------------------------------------------------------------- Throttle
    error_throttle = throttle*0.9;

    // ---------------------------------------------------------------------------------------------- Applying to throttle to escs
    motor_power[0] = ( error_acro_roll) + ( error_acro_pitch) + ( error_acro_yaw);
    motor_power[1] = (-error_acro_roll) + ( error_acro_pitch) + (-error_acro_yaw);
    motor_power[2] = (-error_acro_roll) + (-error_acro_pitch) + ( error_acro_yaw);
    motor_power[3] = ( error_acro_roll) + (-error_acro_pitch) + (-error_acro_yaw);

    // For throttle control logic

    float throttle_value = error_throttle * (~use_vertical_velocity_control & 1) + error_vertical_velocity * use_vertical_velocity_control;
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

void handle_radio_communication(){
    if(!nrf24_data_available(1)){
        return;
    }

    nrf24_receive(rx_data);
    uint8_t rx_data_size = strlen(rx_data);
    // Get the type of request
    extract_request_type(rx_data, rx_data_size, rx_type);

    radio2 = DWT->CYCCNT;
    if(strcmp(rx_type, "js") == 0){
        // printf("JOYSTICK\n");
        last_signal_timestamp_microseconds = get_absolute_time();

        extract_joystick_request_values_float(rx_data, rx_data_size, &throttle, &yaw, &roll, &pitch);

        // Reversed
        roll = (-(roll-50.0))+50.0;

        // For the blackbox log
        remote_control[0] = roll-50.0;
        remote_control[1] = pitch-50.0;
        remote_control[2] = yaw-50.0;
        remote_control[3] = throttle+100.0;

        // the controls were inverse

        // If pitch and roll are neutral then try holding position with gps
        if(pitch == 50.0 && roll == 50.0 && gps_fix_type == 3 && use_gps_hold == 1 && gps_target_set == 0){
            gps_position_hold_enabled = 1;
            // Current gps position is the target now
            target_latitude = gps_latitude;
            target_longitude = gps_longitude;
            gps_target_set = 1;
            // printf("GPS STATE SET\n");
        }else if (pitch != 50.0 || roll != 50.0 ){
            if(use_gps_reset_count >= use_gps_reset_count_to_deactivate){
                gps_position_hold_enabled = 0;
                gps_target_set = 0;
                // printf("GPS STATE RESET, GPS FIX %d\n", gps_fix_type);
                use_gps_reset_count = 0;
            }else{
                use_gps_reset_count++;
            }
        }

        // The remote control itself handles the deadband stuff

        // Roll ##################################################################################################################
        if(roll == 50.0){
            acro_target_roll = 0.0;
            angle_target_roll = 0.0;
        }else {
            acro_target_roll = map_value(roll, 0.0, 100.0, -acro_mode_rate_degrees_per_second, acro_mode_rate_degrees_per_second);
            angle_target_roll = map_value(roll, 0.0, 100.0, -max_roll_attack, max_roll_attack);
        }
        // Pitch ##################################################################################################################
        if(pitch == 50.0){
            acro_target_pitch = 0.0;
            angle_target_pitch = 0.0;
        }else{   
            acro_target_pitch = map_value(pitch, 0.0, 100.0, -acro_mode_rate_degrees_per_second, acro_mode_rate_degrees_per_second);
            angle_target_pitch = map_value(pitch, 0.0, 100.0, -max_pitch_attack, max_pitch_attack);
        }

        // if(flight_mode == 0){
        //     printf("Acro targets: %.2f %.2f %.2f\n", acro_target_roll, acro_target_pitch, acro_target_yaw);
        // }else if(flight_mode == 1){
        //     printf("Angle targets: %.2f %.2f\n", angle_target_roll, angle_target_pitch);
        // }

        // Yaw ##################################################################################################################
        if(yaw == 50) acro_target_yaw = 0;
        else acro_target_yaw = map_value(yaw, 0.0, 100.0, -max_yaw_attack, max_yaw_attack);

        // Throttle ##################################################################################################################
        target_vertical_velocity = map_value(throttle, 0, 100, -max_throttle_vertical_velocity, max_throttle_vertical_velocity);
    }else if(strcmp(rx_type, "pid") == 0){
        printf("\nGot pid change");
        
        float added_proportional = 0;
        float added_integral = 0;
        float added_derivative = 0;
        float added_master_gain = 0;

        extract_pid_request_values(rx_data, strlen(rx_data), &added_proportional, &added_integral, &added_derivative, &added_master_gain);

        // if(!use_angle_mode){ // ACRO MODE
            acro_roll_pitch_gain_p = BASE_ACRO_roll_pitch_GAIN_P + added_proportional;
            acro_roll_pitch_gain_i = BASE_ACRO_roll_pitch_GAIN_I + added_integral;
            acro_roll_pitch_gain_d = BASE_ACRO_roll_pitch_GAIN_D + added_derivative;
            acro_roll_pitch_master_gain = BASE_ACRO_roll_pitch_MASTER_GAIN + added_master_gain;

            // Configure the roll pid 
            pid_set_proportional_gain(&acro_roll_pid, acro_roll_pitch_gain_p * acro_roll_pitch_master_gain);
            pid_set_integral_gain(&acro_roll_pid, acro_roll_pitch_gain_i * acro_roll_pitch_master_gain);
            pid_set_derivative_gain(&acro_roll_pid, acro_roll_pitch_gain_d * acro_roll_pitch_master_gain);
            pid_reset_integral_sum(&acro_roll_pid);

            // Configure the pitch pid 
            pid_set_proportional_gain(&acro_pitch_pid, acro_roll_pitch_gain_p * acro_roll_pitch_master_gain);
            pid_set_integral_gain(&acro_pitch_pid, acro_roll_pitch_gain_i * acro_roll_pitch_master_gain);
            pid_set_derivative_gain(&acro_pitch_pid, acro_roll_pitch_gain_d * acro_roll_pitch_master_gain);
            pid_reset_integral_sum(&acro_pitch_pid);
        // }

        // if(use_angle_mode){ // ANGLE MODE
        //     angle_roll_pitch_gain_p = BASE_ANGLE_roll_pitch_GAIN_P + added_proportional;
        //     angle_roll_pitch_gain_i = BASE_ANGLE_roll_pitch_GAIN_I + added_integral;
        //     angle_roll_pitch_gain_d = BASE_ANGLE_roll_pitch_GAIN_D + added_derivative;
        //     angle_roll_pitch_master_gain = BASE_ANGLE_roll_pitch_MASTER_GAIN + added_master_gain;

        //     added_angle_roll_pitch_gain_p = added_proportional;
        //     added_angle_roll_pitch_gain_i = added_integral;
        //     added_angle_roll_pitch_gain_d = added_derivative;
        //     added_angle_roll_pitch_master_gain = added_master_gain;

        //     // Configure the pitch pid 
        //     pid_set_proportional_gain(&angle_pitch_pid, angle_roll_pitch_gain_p * angle_roll_pitch_master_gain);
        //     pid_set_integral_gain(&angle_pitch_pid, angle_roll_pitch_gain_i * angle_roll_pitch_master_gain);
        //     pid_set_derivative_gain(&angle_pitch_pid, angle_roll_pitch_gain_d * angle_roll_pitch_master_gain);
        //     pid_reset_integral_sum(&angle_pitch_pid);

        //     // Configure the roll pid 
        //     pid_set_proportional_gain(&angle_roll_pid, angle_roll_pitch_gain_p * angle_roll_pitch_master_gain);
        //     pid_set_integral_gain(&angle_roll_pid, angle_roll_pitch_gain_i * angle_roll_pitch_master_gain);
        //     pid_set_derivative_gain(&angle_roll_pid, angle_roll_pitch_gain_d * angle_roll_pitch_master_gain);
        //     pid_reset_integral_sum(&angle_roll_pid);
        // }


        // Start new log blackbox log file if one was running.
        if(sd_card_initialized && use_blackbox_logging){
            // Add end of file to the current file
            uint16_t string_length = 0;
            char *betaflight_end_file_string = betaflight_blackbox_get_end_of_log(&string_length);

            char* sd_card_buffer = (uint8_t*)sd_card_get_buffer_pointer(1);
            uint16_t sd_card_buffer_index = 0;
            uint16_t betaflight_data_string_index = 0;
            while(sd_card_buffer_index < string_length){
                sd_card_buffer[sd_card_buffer_index] = betaflight_end_file_string[sd_card_buffer_index];
                sd_card_buffer_increment_index();
                sd_card_buffer_index++;
                betaflight_data_string_index++;
            }
            free(betaflight_end_file_string);



            if(use_simple_async){
                sd_special_write_chunk_of_string_data_no_slave_response(sd_card_get_buffer_pointer(1));
                sd_buffer_clear(1);
            }else{
                if(use_blackbox_logging){
                    sd_special_write_chunk_of_byte_data_async(sd_card_get_buffer_pointer(1), string_length);
                    sd_buffer_swap();
                    sd_buffer_clear(1);
                }else{
                    sd_special_write_chunk_of_string_data_async(sd_card_get_buffer_pointer(1));
                    sd_buffer_swap();
                    sd_buffer_clear(1);
                }
            }

            printf("SD logging: Sent file ending\n");

            printf("SD logging: Waiting for writing and slave ready\n");
            sd_card_wait_for_dma_transfer_complete();
            sd_card_wait_for_slave_ready();
            if(write_repeat_logs_in_same_file){
                sd_card_wait_for_dma_transfer_complete();
                sd_card_wait_for_slave_ready();

                uint8_t* sd_card_buffer = sd_card_get_buffer_pointer(1);
                uint16_t betaflight_header_length = 0;
                betaflight_blackbox_wrapper_get_header(
                    REFRESH_RATE_HZ,
                    acro_roll_pitch_gain_p,
                    acro_roll_pitch_gain_i,
                    acro_roll_pitch_gain_d,
                    acro_roll_pitch_gain_p,
                    acro_roll_pitch_gain_i,
                    acro_roll_pitch_gain_d,
                    acro_yaw_gain_p,
                    acro_yaw_gain_i,
                    0,
                    angle_roll_pitch_gain_p,
                    angle_roll_pitch_gain_i,
                    angle_roll_pitch_gain_d,
                    filter_gyro_z_yaw_cutoff_frequency,
                    25, // Integral windup
                    21, // Gyro lowpass cutoff
                    20, // Accelerometer lowpass cutoff
                    dshot_refresh_rate,
                    min_dshot600_throttle_value,
                    max_dshot600_throttle_value,
                    actual_min_dshot600_throttle_value,
                    actual_max_dshot600_throttle_value,
                    d_term_filtering_expo,
                    rpm_notch_filter_q_harmonic_1,
                    rpm_notch_filter_min_frequency,
                    d_term_filtering_min_cutoff,
                    d_term_filtering_max_cutoff,
                    d_term_filtering_expo,
                    &betaflight_header_length,
                    sd_card_buffer,
                    10000
                );
                
                sd_card_buffer_increment_index_by_amount(betaflight_header_length);
                HAL_Delay(1000);
                sd_special_write_chunk_of_byte_data_no_slave_response(sd_card_get_buffer_pointer(1), betaflight_header_length);

                sd_buffer_swap();
                sd_buffer_clear_index(1); // Reset the index

                HAL_Delay(1000); // wait a bit for the logger to write the data into SD
                printf("SD logging: sent blackbox header\n");




                // __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
                // __HAL_TIM_SET_COUNTER(&htim4, 0);
                // startup_time_microseconds = get_absolute_time();
                // loop_start_miliseconds = 0;
                // startup_time_microseconds = 0;
            }else{

                
                // Exit async mode. This will close the current file also
                uint64_t i = 0;
                while(1){
                    // printf("%lu\n", i);
                    HAL_Delay(20);
                    if(sd_special_leave_async_mode()){
                        printf("SD logging: Exit async mode request\n");
                        break;
                    }
                    i++;
                }
                printf("SD logging: Left async mode\n");

                HAL_Delay(7000);
                printf("SD logging: Waited 7 s\n");


                // Initialize new_file
                setup_logging_to_sd(1);
                printf("SD logging: Initialized logging again\n");

                // Reset the loop state
                // loop_iteration = 1;
                // absolute_microseconds_since_start = 0;
                // last_signal_timestamp_microseconds = 1000000;
                // __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
                // __HAL_TIM_SET_COUNTER(&htim4, 0);
                // startup_time_microseconds = get_absolute_time();
                // loop_start_microseconds = 0;
                // loop_start_miliseconds = 0;
                // startup_time_microseconds = 0;
            }

            // Capture any radio messages that were sent durring the boot proccess and discard them
            for (uint8_t i = 0; i < 50; i++){
                if(nrf24_data_available(1)){
                    nrf24_receive(rx_data);
                }
            }

            // Reset the loop state
            // loop_iteration = 1;
            // absolute_microseconds_since_start = 0;
            // loop_start_microseconds = 0;
            // loop_start_miliseconds = 0;

            // startup_time_microseconds = 0;
            // delta_time = 0;
            // time_since_startup_hours = 0;
            // time_since_startup_minutes = 0;
            // time_since_startup_seconds = 0;
            // time_since_startup_ms = 0;
            // time_since_startup_microseconds = 0;

            // // Reset the timer
            // __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
            // __HAL_TIM_SET_COUNTER(&htim4, 0);
            // startup_time_microseconds = get_absolute_time();
            // entered_loop = 1;

            // loop_start_microseconds = __HAL_TIM_GET_COUNTER(&htim4);
            // loop_start_miliseconds = HAL_GetTick();

            // HAL_Delay((uint16_t)(minimum_signal_timing_seconds * 1000));
        }
        

    }else if(strcmp(rx_type, "remoteSyncBase") == 0){
        printf("\nGot remoteSyncBase");
        send_pid_base_info_to_remote();
    }else if(strcmp(rx_type, "remoteSyncAdded") == 0){
        printf("\nGot remoteSyncAdded");
        send_pid_added_info_to_remote();
    }else if(strcmp(rx_type, "accel") == 0){

        float added_x_axis_offset;
        float added_y_axis_offset;

        extract_accelerometer_offsets(rx_data, strlen(rx_data), &added_x_axis_offset, &added_y_axis_offset);
        printf("\nGot offsets %f %f ", -added_x_axis_offset, -added_y_axis_offset);

        accelerometer_roll_offset = base_accelerometer_roll_offset - added_x_axis_offset;
        accelerometer_pitch_offset = base_accelerometer_pitch_offset -  added_y_axis_offset;

        pid_reset_integral_sum(&angle_roll_pid);
        pid_reset_integral_sum(&angle_pitch_pid);

        calibrate_gyro(); // Recalibrate the gyro as the temperature affects the calibration
        printf("Calibrated gyro\n");
        // Get the initial position again
        get_initial_position();
        printf("Got initial position again\n");

        // Capture any radio messages that were sent durring the calibration proccess and discard them
        for (uint8_t i = 0; i < 50; i++){
            if(nrf24_data_available(1)){
                nrf24_receive(rx_data);
            }
        }
        printf("Got rid of old radio values\n");
        
        // Reset the sesor fusion and the pid accumulation
    }else if(strcmp(rx_type, "fm") == 0){
        printf("\nGot flight mode");

        uint8_t new_flight_mode;

        extract_flight_mode(rx_data, strlen(rx_data), &new_flight_mode);

        printf("\nGot new flight mode %d\n", new_flight_mode);
        flight_mode = new_flight_mode;

        set_flight_mode(flight_mode);

        find_accelerometer_error(10);

    }

    rx_type[0] = '\0'; // Clear out the string by setting its first char to string terminator
}


void initialize_control_abstractions(){
    acro_roll_pid = pid_init(acro_roll_pitch_master_gain * acro_roll_pitch_gain_p, acro_roll_pitch_master_gain * acro_roll_pitch_gain_i, acro_roll_pitch_master_gain * acro_roll_pitch_gain_d, 0.0, get_absolute_time(), 25, -25, 1);
    acro_pitch_pid = pid_init(acro_roll_pitch_master_gain * acro_roll_pitch_gain_p, acro_roll_pitch_master_gain * acro_roll_pitch_gain_i, acro_roll_pitch_master_gain * acro_roll_pitch_gain_d, 0.0, get_absolute_time(), 25, -25, 1);
    acro_yaw_pid = pid_init(acro_yaw_gain_p, acro_yaw_gain_i, 0, 0.0, get_absolute_time(), 10.0, -10.0, 1);

    angle_roll_pid = pid_init(angle_roll_pitch_master_gain * angle_roll_pitch_gain_p, angle_roll_pitch_master_gain * angle_roll_pitch_gain_i, angle_roll_pitch_master_gain * angle_roll_pitch_gain_d, 0.0, get_absolute_time(), acro_mode_rate_degrees_per_second, -acro_mode_rate_degrees_per_second, 1);
    angle_pitch_pid = pid_init(angle_roll_pitch_master_gain * angle_roll_pitch_gain_p, angle_roll_pitch_master_gain * angle_roll_pitch_gain_i, angle_roll_pitch_master_gain * angle_roll_pitch_gain_d, 0.0, get_absolute_time(), acro_mode_rate_degrees_per_second, -acro_mode_rate_degrees_per_second, 1);

    vertical_velocity_pid = pid_init(vertical_velocity_gain_p, vertical_velocity_gain_i, vertical_velocity_gain_d, 0.0, get_absolute_time(), 90.0, 0.0, 1); // Min value is 0 because motors dont go lower that that

    gps_longitude_pid = pid_init(gps_hold_gain_p, gps_hold_gain_i, gps_hold_gain_d, 0.0, get_absolute_time(), gps_pid_angle_of_attack_max, -gps_pid_angle_of_attack_max, 1);
    gps_latitude_pid = pid_init(gps_hold_gain_p, gps_hold_gain_i, gps_hold_gain_d, 0.0, get_absolute_time(), gps_pid_angle_of_attack_max, -gps_pid_angle_of_attack_max, 1);

    filter_magnetometer_x = filtering_init_low_pass_filter_pt1(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);
    filter_magnetometer_y = filtering_init_low_pass_filter_pt1(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);
    filter_magnetometer_z = filtering_init_low_pass_filter_pt1(filtering_magnetometer_cutoff_frequency, REFRESH_RATE_HZ);

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

void set_flight_mode(uint8_t mode){
    if(mode == 0){
        use_angle_mode = 0;
        use_vertical_velocity_control = 0;
        use_gps_hold = 0;
    }else if(mode == 1){
        use_angle_mode = 1;
        use_vertical_velocity_control = 0;
        use_gps_hold = 0;
    }else if(mode == 2){
        use_angle_mode = 1;
        use_vertical_velocity_control = 1;
        use_gps_hold = 0;
    }else if(mode == 3){
        use_angle_mode = 1;
        use_vertical_velocity_control = 1;
        use_gps_hold = 1;
    }else if(mode == 4){
        use_angle_mode = 1;
        use_vertical_velocity_control = 0;
        use_gps_hold = 1;
    }
}

void indicate_mistake_on_logger(){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(1000);
}

void indicate_mistake_on_sd_card(){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(200);
}

void indicate_sd_logging_ok(){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(250);
}

void setup_logging_to_sd(uint8_t use_updated_file_name){
    // Initialize sd card logging
    sd_card_initialized = 0;
    if(!use_updated_file_name) sd_card_initialize_spi(&hspi3, GPIOA, GPIO_PIN_15, GPIOA, GPIO_PIN_12);
    

    // OK
    if(!sd_test_interface()){
        printf("Failed to initialize the sd card interface: Logging slave issue\n");
        indicate_mistake_on_logger();
        return;
    }

    printf("SD logging: Logger module is working\n");

    // BLACKBOX
    if(use_blackbox_logging){
        if(use_updated_file_name){
            sd_card_initialized = sd_special_initialize(new_log_file_blackbox_base_name);
        }else{
            // OK
            sd_card_initialized = sd_special_initialize(log_file_blackbox_base_name);
        }


        if(!sd_card_initialized){
            printf("Failed to initialize the sd card interface: SD card connection issue\n");
            printf("SD logging: FAILED to initialize blackbox logging. Code %d\n", sd_get_response());
            indicate_mistake_on_sd_card();
            return;
        }

        printf("SD logging: Initialized blackbox logging. Code %d\n", sd_get_response());


        // Only has to be done initially not when new files are opened after that
        if(sd_card_initialized && !use_updated_file_name){
            // Get the index of the file

            // OK
            file_index = sd_special_get_file_index();
            printf("SD logging: File index is %d\n", file_index);

            // Create the new file name string. Will be used for repeated log starts
            uint16_t length = snprintf(
                NULL, 
                0,
                "-%d%s", 
                file_index,
                log_file_blackbox_base_name
            );
            
            // allocate memory for the string
            new_log_file_blackbox_base_name = (char*)malloc((length + 1) + sizeof(char)); // +1 for the null terminator

            // format the string
            snprintf(
                (char*)new_log_file_blackbox_base_name, 
                length + 1, 
                "-%d%s", 
                file_index,
                log_file_blackbox_base_name
            );
        }

        sd_card_async = sd_special_enter_async_byte_mode(1);// Slave should reset when async stops

        if(!sd_card_async){
            printf("SD logging: failed to enter async byte mode\n");
            indicate_mistake_on_logger();
            sd_card_initialized = 0;
            return;
        }
        printf("SD logging: entered async BLACKBOX mode\n");

        uint8_t* sd_card_buffer = sd_card_get_buffer_pointer(1);
        uint16_t betaflight_header_length = 0;
        betaflight_blackbox_wrapper_get_header(
            REFRESH_RATE_HZ,
            acro_roll_pitch_gain_p,
            acro_roll_pitch_gain_i,
            acro_roll_pitch_gain_d,
            acro_roll_pitch_gain_p,
            acro_roll_pitch_gain_i,
            acro_roll_pitch_gain_d,
            acro_yaw_gain_p,
            acro_yaw_gain_i,
            0,
            angle_roll_pitch_gain_p,
            angle_roll_pitch_gain_i,
            angle_roll_pitch_gain_d,
            filter_gyro_z_yaw_cutoff_frequency,
            25, // Integral windup
            21, // Gyro lowpass cutoff
            20, // Accelerometer lowpass cutoff
            dshot_refresh_rate,
            min_dshot600_throttle_value,
            max_dshot600_throttle_value,
            actual_min_dshot600_throttle_value,
            actual_max_dshot600_throttle_value,
            d_term_filtering_expo,
            rpm_notch_filter_q_harmonic_1,
            rpm_notch_filter_min_frequency,
            d_term_filtering_min_cutoff,
            d_term_filtering_max_cutoff,
            d_term_filtering_expo,
            &betaflight_header_length,
            sd_card_buffer,
            10000
        );
        
        sd_card_buffer_increment_index_by_amount(betaflight_header_length);

        sd_special_write_chunk_of_byte_data_no_slave_response(sd_card_get_buffer_pointer(1), betaflight_header_length);

        sd_buffer_swap();
        sd_buffer_clear_index(1); // Reset the index

        HAL_Delay(1000); // wait a bit for the logger to write the data into SD
        printf("SD logging: sent blackbox header\n");

    // TXT
    }else if(!use_blackbox_logging){
        sd_card_initialized = sd_special_initialize(log_file_base_name);

        if(!sd_card_initialized){
            printf("SD logging: FAILED to initialize txt logging. Code %d\n", sd_get_response());
            printf("Failed to initialize the sd card interface: SD card connection issue\n");
            indicate_mistake_on_sd_card();
            return;
        }

        printf("SD logging: Initialized txt logging. Code %d\n", sd_get_response());

        sd_card_async = sd_special_enter_async_string_mode();

        if(!sd_card_async){
            printf("SD logging: failed to enter async TXT mode\n");
            indicate_mistake_on_logger();
            sd_card_initialized = 0;
            return;
        }
        printf("SD logging: entered async TXT mode\n");
    }
    
    indicate_sd_logging_ok();
}

void handle_pre_loop_start(){
    printf("Looping\n");

    // Capture any radio messages that were sent durring the boot proccess and discard them
    for (uint8_t i = 0; i < 50; i++){
        if(nrf24_data_available(1)){
            nrf24_receive(rx_data);
        }
    }

    // Reset the timer
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    startup_time_microseconds = get_absolute_time();
    entered_loop = 1;

    // Temperature will probably not change durring the flight
    // TODO: Needs more testing for long term stability
    temperature = bmp280_get_temperature_celsius();

}

void handle_loop_start(){
    loop_start_microseconds = __HAL_TIM_GET_COUNTER(&htim4);
    loop_start_miliseconds = HAL_GetTick();
}

uint64_t get_absolute_time(){
    // Get the current updated absolute timestamp
    return absolute_microseconds_since_start + __HAL_TIM_GET_COUNTER(&htim4);
}

#define MICROSECONDS_TO_HOURS (1.0f / 3600000000.0f) // The division is more efficient with multiplication
#define HOURS_TO_MICROSECONDS (3600000000.0f)

#define MICROSECONDS_TO_MINUTES (1.0f / 60000000.0f)
#define MINUTES_TO_MICROSECONDS (60000000.0f)

#define MICROSECONDS_TO_SECONDS (1.0f / 1000000.0f)
#define SECONDS_TO_MICROSECONDS (1000000.0f)

#define MICROSECONDS_TO_MILISECONDS (1.0f / 1000.0f)
#define MILISECONDS_TO_MICROSECONDS (1000.0f)

void handle_logging(){
    delta_time = get_absolute_time() - startup_time_microseconds; // Microseconds
    time_since_startup_hours =                  delta_time * MICROSECONDS_TO_HOURS;
    time_since_startup_minutes =               (delta_time - time_since_startup_hours * HOURS_TO_MICROSECONDS) * MICROSECONDS_TO_MINUTES;
    time_since_startup_seconds =               (delta_time - time_since_startup_hours * HOURS_TO_MICROSECONDS - time_since_startup_minutes * MINUTES_TO_MICROSECONDS) * MICROSECONDS_TO_SECONDS;
    time_since_startup_ms =                    (delta_time - time_since_startup_hours * HOURS_TO_MICROSECONDS - time_since_startup_minutes * MINUTES_TO_MICROSECONDS - time_since_startup_seconds * SECONDS_TO_MICROSECONDS) * MICROSECONDS_TO_MILISECONDS;
    time_since_startup_microseconds =           delta_time - time_since_startup_hours * HOURS_TO_MICROSECONDS - time_since_startup_minutes * MINUTES_TO_MICROSECONDS - time_since_startup_seconds * SECONDS_TO_MICROSECONDS - time_since_startup_ms * MILISECONDS_TO_MICROSECONDS;
    uint32_t time_blackbox = ((time_since_startup_hours * 60 + time_since_startup_minutes * 60) + time_since_startup_seconds) * 1000000 + time_since_startup_ms * 1000 + time_since_startup_microseconds;

    if(sd_card_initialized){
        uint16_t data_size = 0;

        if(use_blackbox_logging){
            uint16_t data_size_data = 0;
            uint8_t* sd_card_buffer = (uint8_t*)sd_card_get_buffer_pointer(1);
            betaflight_blackbox_get_encoded_data_string(
                loop_iteration,
                time_blackbox,
                PID_proportional,
                PID_integral,
                PID_derivative,
                PID_feed_forward,
                remote_control,
                PID_set_points,
                gyro_angular,
                acceleration_data,
                motor_power,
                magnetometer_data,
                motor_frequency,
                altitude,
                &data_size_data,
                sd_card_buffer
            );
            
            sd_card_buffer_increment_index_by_amount(data_size);
            
            uint16_t data_size_gps = 0;
            if(got_gps){
                uint16_t data_size_gps = 0;
                betaflight_blackbox_get_encoded_gps_string(
                    bn357_get_utc_time_raw(),
                    bn357_get_fix_quality(),
                    bn357_get_latitude_decimal_format(),
                    bn357_get_longitude_decimal_format(),
                    &data_size_gps, // it will append but not overwrite
                    sd_card_buffer + sd_buffer_size(1) // Offset the buffer, to not overwrite the data
                );
                sd_card_buffer_increment_index_by_amount(data_size);
            }
            data_size = data_size_data + data_size_gps;
        }else{
            // TXT based logging
            // Log a bit of data
            // sd_card_append_to_buffer(1, "%02d:%02d:%02d:%03d;", time_since_startup_hours, time_since_startup_minutes, time_since_startup_seconds, time_since_startup_ms);
            // sd_card_append_to_buffer(1, "ACCEL,%.2f,%.2f,%.2f;", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
            // sd_card_append_to_buffer(1, "GYRO,%.2f,%.2f,%.2f;", imu_orientation[0], imu_orientation[1], imu_orientation[2]);
            // sd_card_append_to_buffer(1, "MAG,%.2f,%.2f,%.2f;", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);
            // sd_card_append_to_buffer(1, "MOTOR,1=%.2f,2=%.2f,3=%.2f,4=%.2f;", motor_power[0], motor_power[1], motor_power[2], motor_power[3]);
            // sd_card_append_to_buffer(1, "ERROR,pitch=%.2f,roll=%.2f,yaw=%.2f,altitude=%.2f;", error_angle_pitch, error_angle_roll, error_acro_yaw, error_altitude);
            // sd_card_append_to_buffer(1, "TEMP,%.2f;", temperature);
            // sd_card_append_to_buffer(1, "ALT %.2f;", altitude);
            // sd_card_append_to_buffer(1, "GPS,lon-%f,lat-%f;", bn357_get_longitude_decimal_format(), bn357_get_latitude_decimal_format());
            // sd_card_append_to_buffer(1, "\n");

            // sd_card_append_to_buffer(1, "%6.5f %6.5f %6.5f", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);  
            // sd_card_append_to_buffer(1, "\n");
        } 
        
        if(sd_card_async){
            if(use_simple_async){
                sd_special_write_chunk_of_string_data_no_slave_response(sd_card_get_buffer_pointer(1));
                sd_buffer_clear(1);
            }else{
                if(use_blackbox_logging){
                    logging2 = DWT->CYCCNT;
                    
                    sd_special_write_chunk_of_byte_data_async(sd_card_get_buffer_pointer(1), data_size);
                    // uint8_t * buffer_new = (uint8_t *)sd_card_get_buffer_pointer(1);
                    // printf("%d write  ", data_size);
                    // for (uint16_t i = 0; i < data_size; i++){
                    //     printf("%02X ", buffer_new[i]);
                    // }
                    // printf("\n");
                    sd_buffer_swap();
                    sd_buffer_clear_index(1); // Reset the index
                    // sd_buffer_clear(1); 


                    
                    // printf("d%d\n", data_size);

                    logging3 = DWT->CYCCNT;
                }else{
                    sd_special_write_chunk_of_string_data_async(sd_card_get_buffer_pointer(1));
                    sd_buffer_swap();
                    sd_buffer_clear(1);
                }
            }
        }else{
            if(use_blackbox_logging){
                sd_card_initialized = sd_special_write_chunk_of_byte_data(sd_card_get_buffer_pointer(1), data_size);
            }else{
                sd_card_initialized = sd_special_write_chunk_of_string_data(sd_card_get_buffer_pointer(1));
            }
        }
    }

    // printf("Motor BL: %5.1f BR: %5.1f FR: %5.1f FL: %5.1f \n",
    //     motor_frequency[0],
    //     motor_frequency[1],
    //     motor_frequency[2],
    //     motor_frequency[3]
    // );

    // printf("%5.1f\n",
    //     motor_frequency[0]
    // );

    // printf("ACCEL,%.2f,%.2f,%.2f;", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
    // printf("GYRO,%5.2f,%5.2f,%5.2f;", gyro_angular[0], gyro_angular[1], gyro_angular[2]);
    // printf("MAG,%.2f,%.2f,%.2f;", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);
    // printf("TEMP,%.2f;", temperature);
    // printf("ALT %.2f;", altitude);
    // printf("\n");

    // Update the blue led with current sd state
    if(sd_card_initialized){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }else{
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }
}


void handle_loop_end(){
    loop_iteration++;

    // printf("b%.1f\n", (float)(__HAL_TIM_GET_COUNTER(&htim4) - loop_start_microseconds)/1000.0);
    // printf("%.1f\n", (float)(__HAL_TIM_GET_COUNTER(&htim4) - loop_start_microseconds)/1000.0);
    // printf("%.2f\n", (float)(__HAL_TIM_GET_COUNTER(&htim4) - loop_start_microseconds)/1000.0);

    // If the delta is less than the target keep looping
    while (precalculated_timing_miliseconds > (__HAL_TIM_GET_COUNTER(&htim4) - loop_start_microseconds));

    // Get the final count and add it to the total microsecond
    // If the timer overflew use the ticks instead

    // Right now this HAL_GetTick method is broken as the HAL_GetTick returns a totaly innacurate value
    // TICK 1961427072.000000 = 1762787968.000000 + ((228252.000000 - 29613.000000) * 1000)

    if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE)){
        // float temp_absl = (float)absolute_microseconds_since_start;
        absolute_microseconds_since_start = absolute_microseconds_since_start + 65536;
        // printf("TICK %f = %f + 65536\n", (float)absolute_microseconds_since_start, temp_absl);
        
        // The bellow is currently not working.

        // absolute_microseconds_since_start = absolute_microseconds_since_start + ((HAL_GetTick() - loop_start_miliseconds) * 1000);
        // printf("TICK %f = %f + ((%f - %f) * 1000)\n", (float)absolute_microseconds_since_start, temp_absl, (float)HAL_GetTick(), (float)loop_start_miliseconds);
        // printf("a%lu\n", HAL_GetTick() - loop_start_miliseconds);

    }else{
        //
        // float temp_absl = (float)absolute_microseconds_since_start;
        absolute_microseconds_since_start = absolute_microseconds_since_start + (__HAL_TIM_GET_COUNTER(&htim4) - loop_start_microseconds);
        // printf("NORM %f = %f + (%f - %f)\n", (float)absolute_microseconds_since_start, temp_absl, (float)__HAL_TIM_GET_COUNTER(&htim4), (float)loop_start_microseconds);
        // printf("a%.1f\n", (float)(__HAL_TIM_GET_COUNTER(&htim4) - loop_start_microseconds)/1000.0);
    }

    // Reset the counter
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    // printf("end %lu\n", absolute_microseconds_since_start);
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

void init_STM32_peripherals(){
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    HAL_Delay(1);
    MX_DMA_Init(); // This has to be before the uart inits, otherwise dma interrupts dont work
    I2C_Bus_Reset(GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7); // Check and reset the i2c slaves if they are fucked up after reflash or restart
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_SPI3_Init();
    HAL_Delay(1);
    MX_USART2_UART_Init();
    MX_USART1_UART_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    RetargetInit(&huart1);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    // Initialize timer 4
    HAL_TIM_Base_Start(&htim4);
}

void check_calibrations(){
    HAL_Delay(2000);
    // Checking errors of mpu6050
    find_accelerometer_error(1000);
    find_gyro_error(1000);
}

void calibrate_gyro(){
    // Gyro can be calibrated when it is standing still. Not necessarily on flat ground.
    float gyro_correction_temp[3] = {
        0, 0, 0
    };  

    find_and_return_gyro_error(1000, gyro_correction_temp);
    mpu6050_apply_calibrations_gyro(gyro_correction_temp);
}



void get_initial_position(){
    // Find the initial position in degrees and apply it to the gyro measurement integral
    // This will tell the robot which way to go to get the actual upward
    __disable_irq();
    mmc5603_magnetometer_readings_micro_teslas(magnetometer_data);
    mpu6050_get_accelerometer_readings_gravity(acceleration_data);
    mpu6050_get_gyro_readings_dps(gyro_angular);
    __enable_irq();

    invert_axies(acceleration_data);
    invert_axies(gyro_angular);
    rotate_magnetometer_output_90_degrees_anti_clockwise(magnetometer_data);

    magnetometer_data[0] = low_pass_filter_pt1_read(&filter_magnetometer_x, magnetometer_data[0]);
    magnetometer_data[1] = low_pass_filter_pt1_read(&filter_magnetometer_y, magnetometer_data[1]);
    magnetometer_data[2] = low_pass_filter_pt1_read(&filter_magnetometer_z, magnetometer_data[2]);

    calculate_roll_pitch_from_accelerometer_data(acceleration_data, &accelerometer_roll, &accelerometer_pitch, accelerometer_roll_offset, accelerometer_pitch_offset);
    calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data, &magnetometer_yaw, accelerometer_roll, accelerometer_pitch, yaw_offset);

    imu_orientation[0] = accelerometer_roll;
    imu_orientation[1] = accelerometer_pitch;
    imu_orientation[2] = magnetometer_yaw;
    printf("Initial imu orientation x: %.2f y: %.2f, z: %.2f\n", imu_orientation[0], imu_orientation[1], imu_orientation[2]);
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

float mapValue(float value, float input_min, float input_max, float output_min, float output_max) {
    // Calculate the input and output ranges' lengths
    float input_range = input_max - input_min;
    float output_range = output_max - output_min;

    // Normalize the input value relative to the input range
    float normalized_value = (value - input_min) / input_range;

// Scale the normalized value according to the output range
    float scaled_value = normalized_value * output_range;

    // Add the output range's minimum value to the scaled value
    float output_value = output_min + scaled_value;

    return output_value;
}

float get_sensor_fusion_altitude(float gps_altitude, float barometer_altitude){
    return gps_altitude + barometer_altitude;
}

// Extract the values form a slash separated stirng into specific variables for motion control parameters 
void extract_joystick_request_values_uint(char *request, uint8_t request_size, uint8_t *throttle, uint8_t *yaw, uint8_t *roll, uint8_t *pitch)
{
    // Skip the request type
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;
    char throttle_string[length + 1];
    strncpy(throttle_string, start, length);
    throttle_string[length] = '\0';
    *throttle = atoi(throttle_string);
    //printf("'%s'\n", throttle_string);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char yaw_string[length + 1];
    strncpy(yaw_string, start, length);
    yaw_string[length] = '\0';
    *yaw = atoi(yaw_string);
    //printf("'%s'\n", yaw_string);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char roll_string[length + 1];
    strncpy(roll_string, start, length);
    roll_string[length] = '\0';
    *roll = atoi(roll_string);
    //printf("'%s'\n", roll_string);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char pitch_string[length + 1];
    strncpy(pitch_string, start, length);
    pitch_string[length] = '\0';
    *pitch = atoi(pitch_string);
    //printf("'%s'\n", pitch_string);
}

// Extract the values from a slash-separated string into specific variables for motion control parameters
void extract_joystick_request_values_float(char *request, uint8_t request_size, float *throttle, float *yaw, float *roll, float *pitch) {
    // Skip the request type
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');

    // Parse throttle
    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;
    char throttle_string[length + 1];
    strncpy(throttle_string, start, length);
    throttle_string[length] = '\0';
    *throttle = atof(throttle_string);

    // Parse yaw
    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char yaw_string[length + 1];
    strncpy(yaw_string, start, length);
    yaw_string[length] = '\0';
    *yaw = atof(yaw_string);

    // Parse roll
    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char roll_string[length + 1];
    strncpy(roll_string, start, length);
    roll_string[length] = '\0';
    *roll = atof(roll_string);

    // Parse pitch
    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char pitch_string[length + 1];
    strncpy(pitch_string, start, length);
    pitch_string[length] = '\0';
    *pitch = atof(pitch_string);
}

// Extract specifically the request type from a slash separated string
void extract_request_type(char *request, uint8_t request_size, char *type_output){
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;

    // You better be sure the length of the output string is big enough
    strncpy(type_output, start, length);
    type_output[length] = '\0';
    // printf("'%s'\n", type_output);
}

// Extract the values form a slash separated stirng into specific variables for pid control parameters 
void extract_pid_request_values(char *request, uint8_t request_size, float *added_proportional, float *added_integral, float *added_derivative, float *added_master){
    // Skip the request type
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;
    char added_proportional_string[length + 1];
    strncpy(added_proportional_string, start, length);
    added_proportional_string[length] = '\0';
    *added_proportional = strtod(added_proportional_string, NULL);
    //printf("'%s'\n", added_proportional);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char added_integral_string[length + 1];
    strncpy(added_integral_string, start, length);
    added_integral_string[length] = '\0';
    *added_integral = strtod(added_integral_string, NULL);
    //printf("'%s'\n", added_integral);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char added_derivative_string[length + 1];
    strncpy(added_derivative_string, start, length);
    added_derivative_string[length] = '\0';
    *added_derivative = strtod(added_derivative_string, NULL);
    //printf("'%s'\n", added_derivative);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char added_master_string[length + 1];
    strncpy(added_master_string, start, length);
    added_master_string[length] = '\0';
    *added_master = strtod(added_master_string, NULL);
    //printf("'%s'\n", added_master);
}

void extract_accelerometer_offsets(char *request, uint8_t request_size, float *added_x_axis_offset, float *added_y_axis_offset){
    // Skip the request type
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;
    char added_x_axis_offset_string[length + 1];
    strncpy(added_x_axis_offset_string, start, length);
    added_x_axis_offset_string[length] = '\0';
    *added_x_axis_offset = strtod(added_x_axis_offset_string, NULL);
    //printf("'%s'\n", added_x_axis_offset);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char added_y_axis_offset_string[length + 1];
    strncpy(added_y_axis_offset_string, start, length);
    added_y_axis_offset_string[length] = '\0';
    *added_y_axis_offset = strtod(added_y_axis_offset_string, NULL);
    //printf("'%s'\n", added_y_axis_offset);
}

void extract_flight_mode(char *request, uint8_t request_size, uint8_t *new_flight_mode){
    // Skip the request type
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;
    char new_flight_mode_string[length + 1];
    strncpy(new_flight_mode_string, start, length);
    new_flight_mode_string[length] = '\0';
    *new_flight_mode = strtoul(new_flight_mode_string, NULL, 10);
    //printf("'%s'\n", added_x_axis_offset);
}


// Map value from a specified range to a new range
float map_value(float value, float input_min, float input_max, float output_min, float output_max) {
    // Calculate the input and output ranges' lengths
    // Normalize the input value relative to the input range
    // Scale the normalized value according to the output range
    // Add the output range's minimum value to the scaled value
    return output_min + (((value - input_min) / (input_max - input_min)) * (output_max - output_min));
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

// Apply a dead zone to a value 
float apply_dead_zone(float value, float max_value, float min_value, float dead_zone){
    float mid_point = (max_value + min_value) / 2;
    float half_range = (max_value - min_value) / 2;
    float normalized_value = (value - mid_point) / half_range; // this will be -1 at min_value, +1 at max_value

    float dead_zone_normalized = dead_zone / half_range;

    float return_value;

    // remove the deadzone
    if (normalized_value > dead_zone_normalized) {
        return_value = (normalized_value - dead_zone_normalized) / (1.0 - dead_zone_normalized);
    } else if (normalized_value < -dead_zone_normalized) {
        return_value = (normalized_value + dead_zone_normalized) / (1.0 - dead_zone_normalized);
    } else {
        return_value = 0.0;
    }

    return return_value * half_range + mid_point; // scale back to original range
}

// Switch to transmit mode and send out a the base pid values to the remote
void send_pid_base_info_to_remote(){
    nrf24_tx_mode(tx_address, 10);
    // Delay a bit to avoid problem
    HAL_Delay(30);

    // Make a slash separated value
    char *string = generate_message_pid_values_nrf24(
        BASE_ANGLE_roll_pitch_GAIN_P, 
        BASE_ANGLE_roll_pitch_GAIN_I, 
        BASE_ANGLE_roll_pitch_GAIN_D,
        BASE_ANGLE_roll_pitch_MASTER_GAIN
    );
    

    // The remote always receives data as a gibberish with corrupted characters. Sending many of them will mean the remote can reconstruct the message
    // And no crc check did not work, i tried.
    for (size_t i = 0; i < 100; i++)
    {
        if(!nrf24_transmit(string)){
            printf("FAILED\n"); // Very djank
        }
    }
    free(string);
    
    // Switch back to receiver mode
    nrf24_rx_mode(tx_address, 10);
}

// Switch to transmit mode and send out a the added  pid values to the remote
void send_pid_added_info_to_remote(){
    nrf24_tx_mode(tx_address, 10);
    // Delay a bit to avoid problem
    HAL_Delay(30);

    // Make a slash separated value
    char *string = generate_message_pid_values_nrf24(
        added_angle_roll_pitch_gain_p, 
        added_angle_roll_pitch_gain_i, 
        added_angle_roll_pitch_gain_d,
        added_angle_roll_pitch_master_gain
    );
    
    // The remote always receives data as a gibberish with corrupted characters. Sending many of them will mean the remote can reconstruct the message
    // And no crc check did not work, i tried.
    for (size_t i = 0; i < 200; i++)
    {
        if(!nrf24_transmit(string)){
            printf("FAILED\n"); // Very djank
        }
    }
    free(string);
    
    // Switch back to receiver mode
    nrf24_rx_mode(tx_address, 10);
}

char* generate_message_pid_values_nrf24(float base_proportional, float base_integral, float base_derivative, float base_master){
    // calculate the length of the resulting string

    // the s is there for reasons... I just ran out of space on the 32 byte buffer for sending. It was originally supposed to be a full name
    int length = snprintf(
        NULL, 
        0,
        "/s/%.2f/%.2f/%.2f/%.2f/  ", 
        base_proportional, 
        base_integral, 
        base_derivative, 
        base_master
    );
    
    // allocate memory for the string
    char *string = malloc(length + 1); // +1 for the null terminator

    // format the string
    snprintf(
        (char*)string, 
        length + 1, 
        "/s/%.2f/%.2f/%.2f/%.2f/  ", 
        base_proportional, 
        base_integral, 
        base_derivative, 
        base_master
    );

    return string;
}

void reset_array_data(float *array, uint8_t array_size){
    memset(array, 0, array_size * sizeof(float));

}

void switch_x_and_y_axis(float *data){
    float temp = 0.0;
    temp = data[0];
    // y is x
    data[0] = data[1];
    // x is -y
    data[1] = temp;
}

void invert_axies(float *data){
    // This effectively turns the sensor around 180 degrees
    data[0] = -data[0];
    data[1] = -data[1];
}

float sawtooth_sin(float x_radian){
    return 2.0f * (x_radian*M_DIVIDE_BY_PI - floorf(x_radian*M_DIVIDE_BY_PI + 0.5f));
}

float sawtooth_cos(float x_radian){
    return 2.0f * ((x_radian + M_HALF_PI)*M_DIVIDE_BY_PI - floorf(x_radian*M_DIVIDE_BY_PI + 0.5f));
}

float triangle_wave(float x) {
    // Normalize x to be within the range [0, 2*PI)
    x = fmodf(x, M_PI_2);
    
    // Scale the normalized x to the range [0, 4]
    float scaled_x = x  * M_DIVIDE_BY_HALF_PI;
    
    // Triangle wave calculation
    if (scaled_x < 1.0f) return scaled_x;
    else if (scaled_x < 3.0f) return 2.0f - scaled_x;
    else return scaled_x - 4.0f;
}

float triangle_sin(float x){
    return triangle_wave(x);
}

float triangle_cos(float x){
    return triangle_wave(x + M_HALF_PI);
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

















/* Auto generated shit again-----------------------------------------------*/

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 1000000; // Max i tested was 1600000 and everything worked. But i am afraid to run it that high
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; // Max possible with DMA slave
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = (1000000/dshot_refresh_rate)-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 100-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
