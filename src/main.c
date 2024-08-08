/* Auto generated shit -----------------------------------------------*/
#include "main.h"
// #include "./FATFS/App/fatfs.h"

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_spi3_tx;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* Actual functional code -----------------------------------------------*/

#include "../lib/printf/retarget.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

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

void init_STM32_peripherals();
void calibrate_escs();
uint16_t setServoActivationPercent(float percent, uint16_t minValue, uint16_t maxValue);
float mapValue(float value, float input_min, float input_max, float output_min, float output_max);
void extract_request_values(char *request, uint8_t request_size, uint8_t *throttle, uint8_t *yaw, uint8_t *pitch, uint8_t *roll);
float get_sensor_fusion_altitude(float gps_altitude, float barometer_altitude);
u_int8_t init_sensors();
void init_loop_timer();
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
void track_time();
float map_value(float value, float input_min, float input_max, float output_min, float output_max);
float apply_dead_zone(float value, float max_value, float min_value, float dead_zone);
void send_pid_base_info_to_remote();
void send_pid_added_info_to_remote();
char* generate_message_pid_values_nrf24(float base_proportional, float base_integral, float base_derivative, float base_master);
void extract_flight_mode(char *request, uint8_t request_size, uint8_t *new_flight_mode);

void handle_get_and_calculate_sensor_values();
void handle_radio_communication();
void handle_logging();
void handle_pid_and_motor_control();
void setup_logging_to_sd();
void handle_loop_end();
void handle_pre_loop_start();
void initialize_control_abstractions();

// PWM pins
// PA8  - 1 TIM1
// PA11 - 4 TIM1
// PA0  - 1 TIM2
// PA1  - 2 TIM2

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
// PB5  SPI RADIO
// PB4  SPI RADIO
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

/**
 * Timer peripheral clock: 75000000Hz
 * Pre scaler value: 72
 * Counter period: 19997
 * Timer clock divided by pre scaler: 1041666.6666666666Hz
 * Frequency: 52.09114700538414Hz (Actually it is like 50.08Hz)
 * Max duty cycle: 19.19712ms
 * Pwm value range: 0-19997
 * Resolution per pwm step: 0.0009600000000000001ms
 *

 * 
 * The esc is shit so we have different max and min than normal
 * 1.08ms/20ms * 4000(Counter period) = 216
 * 1.94ms/20ms * 4000 = 388
 */

const uint16_t max_esc_pwm_value = 2000;
const uint16_t actual_max_esc_pwm_value = 1917; // (max lipo amp rating / max draw of a bldc motor being used x 4) 0.917 * (max_pwm - min_pwm) + min_pwm = 371.4
const uint16_t min_esc_pwm_value = 1000;
const uint16_t esc_lowest_motor_spin = 1033;

// Sensor corrections #################################################################################

// For calibrating the magnetometer I
// used a method found in MicWro Engr Video:
// https://www.youtube.com/watch?v=MinV5V1ioWg
// Mag field norm at my location is 50.503
// use software called Magneto 1.2


// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
// Convert readings from this site to same units nanoTeslas to microteslas

// When calculating this remember that these values are already
// correcting and that the axis of the magnetometer are switched.

// Magnetometer
float hard_iron_correction[3] = {
    3280.531653,3280.845774,3277.030206
};

float soft_iron_correction[3][3] = {
    {1.180417,-0.000133,-0.002451},
    {-0.000133,1.151005,0.051957},
    {-0.002451,0.051957,1.209149}
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

// handling loop timing ###################################################################################
uint32_t loop_start_time = 0;
uint32_t loop_end_time = 0;
int16_t delta_loop_time = 0;

// Accelerometer values to degrees conversion #############################################################
float accelerometer_x_rotation = 0;
float accelerometer_y_rotation = 0;
float magnetometer_z_rotation = 0;

// Radio config ########################################################################################### 
uint8_t tx_address[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
char rx_data[32];
char rx_type[32];

// PID errors ##############################################################################################
struct pid pitch_pid;
struct pid roll_pid;
struct pid yaw_pid;
struct pid vertical_velocity_pid;

float error_pitch = 0;
float error_roll = 0;
float error_yaw = 0;
float error_throttle = 0;
float error_vertical_velocity = 0;

// Actual PID adjustment for pitch
#define BASE_PITCH_ROLL_MASTER_GAIN 0.4
#define BASE_PITCH_ROLL_GAIN_P 0.0 // 0.35
#define BASE_PITCH_ROLL_GAIN_I 0.0 // 0.0
#define BASE_PITCH_ROLL_GAIN_D 0.0 // 130.0


// Notes for PID
// 1) It is important that the natural state without power of the drone is not stable, otherwise 
// some offset of center off mass or pid desired point needs to introduced
// 2) Remember that the drone when in the air has motors spinning at idle power, just enough 
// to float in the air. If you are testing with drone constrained add a base motor speed to account for this.

// PID for yaw
const float yaw_gain_p = 0.75; 
const float yaw_gain_i = 0.3;

// PID for altitude control
const float vertical_velocity_gain_p = 0.6; 
const float vertical_velocity_gain_i = 0.5;
const float vertical_velocity_gain_d = 0.00;

// Used for smooth changes to PID while using remote control. Do not touch this

float pitch_roll_master_gain = BASE_PITCH_ROLL_MASTER_GAIN; // Dont you just love the STM#2 compiler?
float pitch_roll_gain_p = BASE_PITCH_ROLL_GAIN_P;
float pitch_roll_gain_i = BASE_PITCH_ROLL_GAIN_I;
float pitch_roll_gain_d = BASE_PITCH_ROLL_GAIN_D;

float added_pitch_roll_master_gain = 0;
float added_pitch_roll_gain_p = 0;
float added_pitch_roll_gain_i = 0;
float added_pitch_roll_gain_d = 0;

// Refresh rate ##############################################################################################

// remember that the stm32 is not as fast as the esp32 and it cannot print lines at the same speed
// const float refresh_rate_hz = 400;
#define REFRESH_RATE_HZ 200

// Sensor stuff ##############################################################################################
float complementary_ratio = 1.0 - 1.0/(1.0+(1.0/REFRESH_RATE_HZ)); // Depends on how often the loop runs. 1 second / (1 second + one loop time)
// float complementary_beta = 0.005;
float complementary_beta = 0.0;

float filtering_alpha_accelerometer = 0.1;
float filtering_alpha_gyro = 0.01;
float filtering_alpha_magnetometer = 0.01;
float filtering_alpha_barometer = 0.01;


float acceleration_data[] = {0,0,0};
float gyro_angular[] = {0,0,0};
float gyro_degrees[] = {0,0,0};
float magnetometer_data[] = {0,0,0};
float gps_longitude = 0.0;
float gps_latitude = 0.0;
float pressure = 0.0;
float temperature = 0.0;
float altitude = 0.0;
float vertical_velocity = 0.0;

float target_pitch = 0.0;
float target_roll = 0.0;
float target_yaw = 0.0;
float target_altitude = 0.0;
float target_vertical_velocity = 0.0;

float motor_power[] = {0.0, 0.0, 0.0, 0.0};


// Remote control settings ############################################################################################
float max_yaw_attack = 40.0;

float max_pitch_attack = 10;
float pitch_attack_step = 0.2;

float max_roll_attack = 10;
float roll_attack_step = 0.2;

float max_throttle_vertical_velocity = 30; // cm/second
float min_throttle_vertical_velocity = -30; // cm/second

float throttle = 0.0;
float yaw = 50.0;
float last_yaw = 50.0;
float pitch = 50.0;
float roll = 50.0;

uint8_t slowing_lock = 0;

float last_raw_yaw = 0;
float delta_yaw = 0;

float minimum_signal_timing_seconds = 0.2; // Seconds
uint32_t last_signal_timestamp = 0;


// For deciding which log file to log to
char log_file_base_name[] = "Quadcopter.txt";
char log_file_blackbox_base_name[] = "Quadcopter.BBL";
uint8_t sd_card_initialized = 0;
uint8_t log_loop_count = 0;
uint8_t sd_card_async = 0;
const uint8_t use_simple_async = 0; // 0 is the complex async
const uint8_t use_blackbox_logging = 0;


// Keep track of time in each loop. Since loop start
uint32_t startup_time = 0;
uint32_t delta_time = 0;
uint16_t time_since_startup_ms = 0;
uint8_t time_since_startup_minutes = 0;
uint8_t time_since_startup_seconds = 0;
uint8_t time_since_startup_hours = 0;


// Stuff that is needed blackbox logging 
uint32_t loop_iteration = 1;
float PID_proportional[3];
float PID_integral[3];
float PID_derivative[2];
float PID_feed_forward[3];
float PID_set_points[4];
float remote_control[4];

uint8_t entered_loop = 0;
uint8_t got_gps = 0;

struct low_pass_filter filter_accelerometer_x;
struct low_pass_filter filter_accelerometer_y;
struct low_pass_filter filter_accelerometer_z;

struct low_pass_filter filter_gyro_x;
struct low_pass_filter filter_gyro_y;
struct low_pass_filter filter_gyro_z;

struct low_pass_filter filter_magnetometer_x;
struct low_pass_filter filter_magnetometer_y;
struct low_pass_filter filter_magnetometer_z;

struct low_pass_filter filter_barometer;

struct kalman_filter altitude_and_velocity_kalman;

// Flight modes:
// (0) - The controller can take control of the roll, pitch and yaw axis to help stabilize them
// (1) - The controller can take control of the roll, pitch, yaw and altitude axis to help stabilize

uint8_t flight_mode = 0;

uint8_t use_vertical_velocity_control = 0;

// float base_accelerometer_x_roll_offset = -0.42;
// float base_accelerometer_y_pitch_offset = -2.21;


// float base_accelerometer_x_roll_offset = 0.56;
// float base_accelerometer_y_pitch_offset = -2.48;

float base_accelerometer_x_roll_offset = 0.6 + 4.0;
float base_accelerometer_y_pitch_offset = -2.48 + 0.45;

float accelerometer_x_roll_offset = 0;
float accelerometer_y_pitch_offset = 0;

int main(void){
    init_STM32_peripherals();

    // Turn off the blue led. Will show the status of sd logging later
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    
    printf("STARTING PROGRAM\n"); 
    // calibrate_escs();
    if(init_sensors() == 0){
        return 0; // exit if initialization failed
    }


    HAL_Delay(2000); // Dont want to interfere in calibration

    // check_calibrations();
    calibrate_gyro(); // Recalibrate the gyro as the temperature affects the calibration
    get_initial_position();
    // setup_logging_to_sd(); // Hardware power issue currently
    initialize_control_abstractions();

    handle_pre_loop_start();
    while (1){
        handle_radio_communication();
        handle_get_and_calculate_sensor_values(); // Important do do this right before the pid stuff.
        handle_pid_and_motor_control();
        handle_logging();

        // printf("%.5f %.5f %.5f\n", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);

        printf("%.5f %.5f %.5f\n", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);


        handle_loop_end();
    }
}


void initialize_control_abstractions(){
    pitch_pid = pid_init(pitch_roll_master_gain * pitch_roll_gain_p, pitch_roll_master_gain * pitch_roll_gain_i, pitch_roll_master_gain * pitch_roll_gain_d, 0.0, HAL_GetTick(), 10.0, -10.0, 1);
    roll_pid = pid_init(pitch_roll_master_gain * pitch_roll_gain_p, pitch_roll_master_gain * pitch_roll_gain_i, pitch_roll_master_gain * pitch_roll_gain_d, 0.0, HAL_GetTick(), 10.0, -10.0, 1);
    yaw_pid = pid_init(yaw_gain_p, yaw_gain_i, 0, 0.0, HAL_GetTick(), 10.0, -10.0, 1);
    vertical_velocity_pid = pid_init(vertical_velocity_gain_p, vertical_velocity_gain_i, vertical_velocity_gain_d, 0.0, HAL_GetTick(), 90.0, 0.0, 1);
    
    filter_accelerometer_x = filtering_init_low_pass_filter(filtering_alpha_accelerometer);
    filter_accelerometer_y = filtering_init_low_pass_filter(filtering_alpha_accelerometer);
    filter_accelerometer_z = filtering_init_low_pass_filter(filtering_alpha_accelerometer);

    filter_gyro_x = filtering_init_low_pass_filter(filtering_alpha_gyro);
    filter_gyro_y = filtering_init_low_pass_filter(filtering_alpha_gyro);
    filter_gyro_z = filtering_init_low_pass_filter(filtering_alpha_gyro);

    filter_magnetometer_x = filtering_init_low_pass_filter(filtering_alpha_magnetometer);
    filter_magnetometer_y = filtering_init_low_pass_filter(filtering_alpha_magnetometer);
    filter_magnetometer_z = filtering_init_low_pass_filter(filtering_alpha_magnetometer);

    filter_barometer = filtering_init_low_pass_filter(filtering_alpha_magnetometer);

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

    // float Q_proccess_uncertainty_temp[2][2];
    // matrix_transpose(B_control, 2, 1, Q_proccess_uncertainty_temp);
    // matrix_matrix_multiply(B_control, 2, 1, Q_proccess_uncertainty_temp, 1, 2, Q_proccess_uncertainty);
    // matrix_scalar_multiply(Q_proccess_uncertainty, 2, 2, 10*10, Q_proccess_uncertainty_temp);
    // matrix_copy(Q_proccess_uncertainty_temp, 2, 2, Q_proccess_uncertainty);

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

void setup_logging_to_sd(){
    // Initialize sd card logging
    sd_card_initialize_spi(&hspi3, GPIOA, GPIO_PIN_15, GPIOA, GPIO_PIN_12);
    if(sd_test_interface()){
        printf("SD logging: Logger module is working\n");

        if(use_blackbox_logging){
            sd_card_initialized = sd_special_initialize(log_file_blackbox_base_name);
            if(sd_card_initialized){
                printf("SD logging: Initialized blackbox logging. Code %d\n", sd_get_response());
            }else{
                printf("SD logging: FAILED to initialize blackbox logging. Code %d\n", sd_get_response());
            }
        }else{
            sd_card_initialized = sd_special_initialize(log_file_base_name);
            if(sd_card_initialized){
                printf("SD logging: Initialized txt logging. Code %d\n", sd_get_response());
            }else{
                printf("SD logging: FAILED to initialize txt logging. Code %d\n", sd_get_response());
            }
        }

        if(sd_card_initialized){
            if(use_blackbox_logging){
                sd_card_async = sd_special_enter_async_byte_mode();
                printf("SD logging: entered async byte mode\n");
            }else{
                sd_card_async = sd_special_enter_async_string_mode();
                printf("SD logging: entered async string mode\n");
            }
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
        }else{
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
    }else{
        printf("Failed to initialize the sd card interface\n");
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

    if(use_blackbox_logging && sd_card_initialized){
        uint16_t betaflight_header_length = 0;
        char* betaflight_header = betaflight_blackbox_wrapper_get_header(min_esc_pwm_value, actual_max_esc_pwm_value, &betaflight_header_length);
        HAL_Delay(100); // wait a bit for it to sort its shit out
        sd_special_write_chunk_of_byte_data_no_slave_response(betaflight_header, betaflight_header_length);
        free(betaflight_header);
        HAL_Delay(500); // wait a bit for it to sort its shit out
        printf("SD logging: sent blackbox header\n");
    }
}

void handle_pre_loop_start(){
    printf("Looping\n");
    altitude = 10;

    // Capture any radio messages that were sent durring the boot proccess and discard them
    for (uint8_t i = 0; i < 50; i++){
        if(nrf24_data_available(1)){
            nrf24_receive(rx_data);
        }
    }

    init_loop_timer();
    startup_time = HAL_GetTick();
    entered_loop = 1;
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
    temperature = bmp280_get_temperature_celsius();
    altitude = bmp280_get_height_centimeters_from_reference(0);
    
    if(bn357_get_status_up_to_date(1)) got_gps = 1; // Flag for logging
    else got_gps = 0;

    mpu6050_get_accelerometer_readings_gravity(acceleration_data);
    mpu6050_get_gyro_readings_dps(gyro_angular);
    mmc5603_magnetometer_readings_micro_teslas(magnetometer_data);
    // ------------------------------------------------------------------------------------------------------ Filter the sensor data




    // acceleration_data[0] = low_pass_filter_read(&filter_accelerometer_x, acceleration_data[0]);
    // acceleration_data[1] = low_pass_filter_read(&filter_accelerometer_y, acceleration_data[1]);
    // acceleration_data[2] = low_pass_filter_read(&filter_accelerometer_z, acceleration_data[2]);

    // gyro_angular[0] = low_pass_filter_read(&filter_gyro_x, gyro_angular[0]);
    // gyro_angular[1] = low_pass_filter_read(&filter_gyro_y, gyro_angular[1]);
    // gyro_angular[2] = low_pass_filter_read(&filter_gyro_z, gyro_angular[2]);

    // magnetometer_data[0] = low_pass_filter_read(&filter_gyro_x, magnetometer_data[0]);
    // magnetometer_data[1] = low_pass_filter_read(&filter_gyro_y, magnetometer_data[1]);
    // magnetometer_data[2] = low_pass_filter_read(&filter_gyro_z, magnetometer_data[2]);

    // altitude = low_pass_filter_read(&filter_barometer, altitude); // New data  is so few that filtering is unneeded
    // LOG

    // printf("%.3f;%.3f;%.3f;%.3f;%.3f;%.3f\n", acceleration_data[0], acceleration_data[1], acceleration_data[2], gyro_angular[0], gyro_angular[1], gyro_angular[2]);



    // ------------------------------------------------------------------------------------------------------ Sensor fusion
    calculate_degrees_x_y(acceleration_data, &accelerometer_x_rotation, &accelerometer_y_rotation, accelerometer_x_roll_offset, accelerometer_y_pitch_offset); // Get roll and pitch from the data. I call it x and y. Ranges -90 to 90. 

    calculate_yaw(magnetometer_data, &magnetometer_z_rotation);
    last_raw_yaw = magnetometer_z_rotation; // Save the raw yaw for next loop. No mater if yaw changes or not. Need to know the latest one

    convert_angular_rotation_to_degrees_x_y(gyro_angular, gyro_degrees, accelerometer_x_rotation, accelerometer_y_rotation, HAL_GetTick(), 1);
    calculate_yaw_tilt_compensated(magnetometer_data, &magnetometer_z_rotation, gyro_degrees[0], gyro_degrees[1]);
    gyro_degrees[2] = magnetometer_z_rotation;
    // ------------------------------------------------------------------------------------------------------ Use sensor fused data for more data
    float vertical_acceleration[1] = {mpu6050_calculate_vertical_acceleration_cm_per_second(acceleration_data, gyro_degrees)};
    float vertical_altitude[1] = {altitude};

    kalman_filter_predict(&altitude_and_velocity_kalman, vertical_acceleration);
    kalman_filter_update(&altitude_and_velocity_kalman, vertical_altitude);

    altitude = kalman_filter_get_state(&altitude_and_velocity_kalman)[0][0];
    vertical_velocity = kalman_filter_get_state(&altitude_and_velocity_kalman)[1][0];
    // printf("%f;%f;%f;%f;\n", vertical_acceleration[0], vertical_altitude[0], kalman_filter_get_state(&altitude_and_velocity_kalman)[0][0], kalman_filter_get_state(&altitude_and_velocity_kalman)[1][0]);


}

void handle_radio_communication(){
    if(nrf24_data_available(1)){
        nrf24_receive(rx_data);
        // Get the type of request
        extract_request_type(rx_data, strlen(rx_data), rx_type);

        if(strcmp(rx_type, "js") == 0){
            last_signal_timestamp = HAL_GetTick();

            // extract_joystick_request_values_uint(rx_data, strlen(rx_data), &throttle, &yaw, &roll, &pitch);
            extract_joystick_request_values_float(rx_data, strlen(rx_data), &throttle, &yaw, &roll, &pitch);

            // For the blackbox log
            remote_control[0] = -(roll-50); // Reversed for blackbox log
            remote_control[1] = pitch-50;
            remote_control[2] = yaw-50;
            remote_control[3] = throttle+100;

            // the controls were inverse
            pitch = (-(pitch-50))+50;

            // Throttle does not need to be handled

            // Pitch ##################################################################################################################
            // Check if pitch is neutral
            if(pitch == 50){
                // Had some issues with floats not being zero.
                // If value is between pitch attack and negative pitch attack then just set it to zero
                if(target_pitch < pitch_attack_step/2 && target_pitch > -pitch_attack_step/2){
                    target_pitch = 0.0;
                }
                // If value is above pitch attack values 
                // Start slowing going back to zero
                if(target_pitch > 0 && target_pitch != 0){
                    target_pitch = target_pitch - pitch_attack_step;
                }else if(target_pitch < 0  && target_pitch != 0){
                    target_pitch = target_pitch + pitch_attack_step;
                }
            }else if (pitch != 50){
                // If pitch is not neutral start increasing into some direction.
                target_pitch = target_pitch + map_value(pitch, 0.0, 100.0, -pitch_attack_step, pitch_attack_step);
            }

            // Make sure that the value is in the boundaries
            if(target_pitch > max_pitch_attack){
                target_pitch = max_pitch_attack;
            }else if(target_pitch < -max_pitch_attack){
                target_pitch = -max_pitch_attack;
            }


            // Roll ##################################################################################################################
            // Check if roll is neutral
            if(roll == 50){
                // Had some issues with floats not being zero.
                // If value is between roll attack and negative roll attack then just set it to zero
                if(target_roll < roll_attack_step/2 && target_roll > -roll_attack_step/2){
                    target_roll = 0.0;
                }
                // If value is above roll attack values 
                // Start slowing going back to zero
                if(target_roll > 0 && target_roll != 0){
                    target_roll = target_roll - roll_attack_step;
                }else if(target_roll < 0  && target_roll != 0){
                    target_roll = target_roll + roll_attack_step;
                }
            }else if (roll != 50){
                // If roll is not neutral start increasing into some direction.
                target_roll = target_roll + map_value(roll, 0.0, 100.0, -roll_attack_step, roll_attack_step);
            }

            // Make sure that the value is in the boundaries
            if(target_roll > max_roll_attack){
                target_roll = max_roll_attack;
            }else if(target_roll < -max_roll_attack){
                target_roll = -max_roll_attack;
            }

            // Yaw ##################################################################################################################
            if(yaw != 50){
                // There is an issue with the remote sometimes sending a 0 yaw
                // target_yaw = gyro_degrees[2] + map_value(yaw, 0.0, 100.0, -max_yaw_attack, max_yaw_attack);
                // // handle the switch from -180 to 180 degrees
                // if(target_yaw > 180.0){
                //     target_yaw = target_yaw - 360.0;
                // }else if(target_yaw < -180.0){
                //     target_yaw = target_yaw + 360.0;
                // }

                target_yaw = map_value(yaw, 0.0, 100.0, -max_yaw_attack, max_yaw_attack);
            }else{
                target_yaw = 0;
            }

            // Reset the yaw to the current degrees
            // if(last_yaw != 50 && yaw == 50){
            //     target_yaw = gyro_degrees[2];
            // }

            // last_yaw = yaw;

            // Throttle ##################################################################################################################
            // This is only used for flight mode 1
            target_vertical_velocity = map_value(throttle, 0, 100, min_throttle_vertical_velocity, max_throttle_vertical_velocity);

        }else if(strcmp(rx_type, "pid") == 0){
            printf("\nGot pid");
            
            float added_proportional = 0;
            float added_integral = 0;
            float added_derivative = 0;
            float added_master_gain = 0;

            extract_pid_request_values(rx_data, strlen(rx_data), &added_proportional, &added_integral, &added_derivative, &added_master_gain);

            pitch_roll_gain_p = BASE_PITCH_ROLL_GAIN_P + added_proportional;
            pitch_roll_gain_i = BASE_PITCH_ROLL_GAIN_I + added_integral;
            pitch_roll_gain_d = BASE_PITCH_ROLL_GAIN_D + added_derivative;
            pitch_roll_master_gain = BASE_PITCH_ROLL_MASTER_GAIN + added_master_gain;

            added_pitch_roll_gain_p = added_proportional;
            added_pitch_roll_gain_i = added_integral;
            added_pitch_roll_gain_d = added_derivative;
            added_pitch_roll_master_gain = added_master_gain;

            // Configure the pitch pid 
            pid_set_proportional_gain(&pitch_pid, pitch_roll_gain_p * pitch_roll_master_gain);
            pid_set_integral_gain(&pitch_pid, pitch_roll_gain_i * pitch_roll_master_gain);
            pid_set_derivative_gain(&pitch_pid, pitch_roll_gain_d * pitch_roll_master_gain);
            pid_reset_integral_sum(&pitch_pid);

            // Configure the roll pid 
            pid_set_proportional_gain(&roll_pid, pitch_roll_gain_p * pitch_roll_master_gain);
            pid_set_integral_gain(&roll_pid, pitch_roll_gain_i * pitch_roll_master_gain);
            pid_set_derivative_gain(&roll_pid, pitch_roll_gain_d * pitch_roll_master_gain);
            pid_reset_integral_sum(&roll_pid);

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
            printf("\nGot offsets %f %f ", added_x_axis_offset, added_y_axis_offset);

            accelerometer_x_roll_offset = base_accelerometer_x_roll_offset + added_x_axis_offset;
            accelerometer_y_pitch_offset = base_accelerometer_y_pitch_offset + added_y_axis_offset;

            pid_reset_integral_sum(&roll_pid);
            pid_reset_integral_sum(&pitch_pid);

            calibrate_gyro(); // Recalibrate the gyro as the temperature affects the calibration

            // Get the initial position again
            get_initial_position();

            // Capture any radio messages that were sent durring the calibration proccess and discard them
            for (uint8_t i = 0; i < 50; i++){
                if(nrf24_data_available(1)){
                    nrf24_receive(rx_data);
                }
            }

            // Reset the sesor fusion and the pid accumulation
        }else if(strcmp(rx_type, "fm") == 0){
            printf("\nGot flight mode");

            uint8_t new_flight_mode;

            extract_flight_mode(rx_data, strlen(rx_data), &new_flight_mode);

            printf("\nGot new flight mode %d", new_flight_mode);
            flight_mode = new_flight_mode;

            if(flight_mode == 1){
                use_vertical_velocity_control = 1;
            }else if(flight_mode == 0){
                use_vertical_velocity_control = 0;
            }

            find_accelerometer_error(10);
        }

        rx_type[0] = '\0'; // Clear out the string by setting its first char to string terminator
    }
}

void handle_pid_and_motor_control(){
    // float error_altitude = mapValue(pid_get_error(&altitude_pid, altitude, HAL_GetTick()), -180.0, 180.0, -100.0, 100.0);
    // printf("Altitude error %6.2f ", error_altitude);
    
    // For the robot to do work it needs to be receiving radio signals and at the correct angles, facing up
    if(
        gyro_degrees[0] <  30 && 
        gyro_degrees[0] > -30 && 
        gyro_degrees[1] <  30 && 
        gyro_degrees[1] > -30 && 
        ((float)HAL_GetTick() - (float)last_signal_timestamp) / 1000.0 <= minimum_signal_timing_seconds
    ){
        pid_set_desired_value(&pitch_pid, target_pitch);
        pid_set_desired_value(&roll_pid, target_roll);
        pid_set_desired_value(&yaw_pid, target_yaw);
        pid_set_desired_value(&vertical_velocity_pid, target_vertical_velocity);

        PID_set_points[0] = target_pitch; // Logging
        PID_set_points[1] = target_roll; // Logging
        PID_set_points[2] = target_yaw; // Logging

        // pitch is facing to the sides
        // roll is facing forwards and backwards
        error_pitch = pid_get_error(&pitch_pid, gyro_degrees[0], HAL_GetTick());
        PID_proportional[0] = pid_get_last_proportional_error(&pitch_pid); // Logging
        PID_integral[0] = pid_get_last_integral_error(&pitch_pid); // Logging
        PID_derivative[0] = pid_get_last_derivative_error(&pitch_pid); // Logging

        error_roll = pid_get_error(&roll_pid, gyro_degrees[1], HAL_GetTick());
        PID_proportional[1] = pid_get_last_proportional_error(&roll_pid); // Logging
        PID_integral[1] = pid_get_last_integral_error(&roll_pid); // Logging
        PID_derivative[1] = pid_get_last_derivative_error(&roll_pid); // Logging
        
        error_yaw = pid_get_error(&yaw_pid, gyro_angular[2], HAL_GetTick()); // Logging
        PID_proportional[2] = pid_get_last_proportional_error(&yaw_pid); // Logging
        PID_integral[2] = pid_get_last_integral_error(&yaw_pid); // Logging

        error_vertical_velocity = pid_get_error(&vertical_velocity_pid, vertical_velocity, HAL_GetTick());

        error_throttle = throttle*0.9;

        motor_power[0] = (-error_pitch) +  (-error_roll) + ( error_yaw);
        motor_power[1] = (-error_pitch) +  ( error_roll) + (-error_yaw);
        motor_power[2] = ( error_pitch) +  ( error_roll) + ( error_yaw);
        motor_power[3] = ( error_pitch) +  (-error_roll) + (-error_yaw);

        // For throttle control logic
        motor_power[0] += error_throttle * (~use_vertical_velocity_control & 1) + error_vertical_velocity * use_vertical_velocity_control;
        motor_power[1] += error_throttle * (~use_vertical_velocity_control & 1) + error_vertical_velocity * use_vertical_velocity_control;
        motor_power[2] += error_throttle * (~use_vertical_velocity_control & 1) + error_vertical_velocity * use_vertical_velocity_control;
        motor_power[3] += error_throttle * (~use_vertical_velocity_control & 1) + error_vertical_velocity * use_vertical_velocity_control;

        // Motor B (0) 14460 rpm
        // Motor C (1) 14160 rpm
        // Motor D (2) 14460 rpm
        // Motor A (3) 13740 rpm

        // GPS side BACK
        TIM2->CCR1 = setServoActivationPercent(motor_power[2], esc_lowest_motor_spin, actual_max_esc_pwm_value);
        TIM2->CCR2 = setServoActivationPercent(motor_power[3], esc_lowest_motor_spin, actual_max_esc_pwm_value);

        // No gps side FRONT
        TIM1->CCR1 = setServoActivationPercent(motor_power[0], esc_lowest_motor_spin, actual_max_esc_pwm_value);
        TIM1->CCR4 = setServoActivationPercent(motor_power[1], esc_lowest_motor_spin, actual_max_esc_pwm_value);
        
        // For logging
        motor_power[0] = setServoActivationPercent(motor_power[0], esc_lowest_motor_spin, actual_max_esc_pwm_value); // Logging
        motor_power[1] = setServoActivationPercent(motor_power[1], esc_lowest_motor_spin, actual_max_esc_pwm_value); // Logging
        motor_power[2] = setServoActivationPercent(motor_power[2], esc_lowest_motor_spin, actual_max_esc_pwm_value); // Logging
        motor_power[3] = setServoActivationPercent(motor_power[3], esc_lowest_motor_spin, actual_max_esc_pwm_value); // Logging
    }else{
        // GPS side
        TIM2->CCR1 = setServoActivationPercent(0, min_esc_pwm_value, actual_max_esc_pwm_value);
        TIM2->CCR2 = setServoActivationPercent(0, min_esc_pwm_value, actual_max_esc_pwm_value);

        // No gps side
        TIM1->CCR1 = setServoActivationPercent(0, min_esc_pwm_value, actual_max_esc_pwm_value);
        TIM1->CCR4 = setServoActivationPercent(0, min_esc_pwm_value, actual_max_esc_pwm_value);
        
        // Reset the remote control set points also
        remote_control[0] = 0;
        remote_control[1] = 50;
        remote_control[2] = 50;
        remote_control[3] = 50;

        motor_power[0] = setServoActivationPercent(0, min_esc_pwm_value, actual_max_esc_pwm_value); // Logging
        motor_power[1] = setServoActivationPercent(0, min_esc_pwm_value, actual_max_esc_pwm_value); // Logging
        motor_power[2] = setServoActivationPercent(0, min_esc_pwm_value, actual_max_esc_pwm_value); // Logging
        motor_power[3] = setServoActivationPercent(0, min_esc_pwm_value, actual_max_esc_pwm_value); // Logging

        PID_proportional[0] = 0;
        PID_proportional[1] = 0;
        PID_proportional[2] = 0;

        PID_integral[0] = 0;
        PID_integral[1] = 0;
        PID_integral[2] = 0;

        PID_derivative[0] = 0;
        PID_derivative[1] = 0;

        pid_reset_integral_sum(&pitch_pid);
        pid_reset_integral_sum(&roll_pid);
        pid_reset_integral_sum(&vertical_velocity_pid);
    }
}

void handle_logging(){
    delta_time = HAL_GetTick() - startup_time;
    time_since_startup_hours = delta_time / 3600000;
    time_since_startup_minutes = (delta_time - time_since_startup_hours * 3600000) / 60000;
    time_since_startup_seconds = (delta_time - time_since_startup_hours * 3600000 - time_since_startup_minutes * 60000) / 1000;
    time_since_startup_ms = delta_time - time_since_startup_hours * 3600000 - time_since_startup_minutes * 60000 - time_since_startup_seconds * 1000;

    uint32_t time_blackbox = ((time_since_startup_hours * 60 + time_since_startup_minutes * 60) + time_since_startup_seconds) * 1000000 + time_since_startup_ms * 1000; 

    // Print out for debugging
    // printf("%d:%02d:%02d:%03d;", time_since_startup_hours, time_since_startup_minutes, time_since_startup_seconds, time_since_startup_ms);
    // printf("ACCEL, %6.2f, %6.2f, %6.2f, ", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
    // printf("GYRO, %6.2f, %6.2f, %6.2f, ", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);
    // printf("GY %6.2f %6.2f", gyro_degrees[0], gyro_degrees[1]);

    // printf("MAG, %6.2f, %6.2f, %6.2f, ", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);
    // printf("MOTOR 1=%6.2f 2=%6.2f 3=%6.2f 4=%6.2f, ", motor_power[0], motor_power[1], motor_power[2], motor_power[3]);
    // printf("TEMP %6.5f, ", temperature);
    // printf("ALT %6.2f, ", altitude);
    // printf("GPS %f, %f, ", gps_longitude, gps_latitude);
    // printf("ERROR pitch %6.2f, roll %6.2f, pitch %6.2f, yaw %6.2f, altitude %6.2f, ", error_pitch, error_roll, error_yaw, error_altitude);
    // printf("%6.5f %6.5f %6.5f", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);  
    // printf("\n");


    if(sd_card_initialized){
        uint16_t data_size = 0;

        if(use_blackbox_logging){
            char* betaflight_data_string = betaflight_blackbox_get_encoded_data_string(
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
                gyro_degrees,
                altitude,
                &data_size
            );
            char* sd_card_buffer = sd_card_get_buffer_pointer(1);
            uint16_t sd_card_buffer_index = 0;
            uint16_t betaflight_data_string_index = 0;
            while(sd_card_buffer_index< data_size){
                sd_card_buffer[sd_card_buffer_index] = betaflight_data_string[sd_card_buffer_index];
                sd_card_buffer_increment_index();
                sd_card_buffer_index++;
                betaflight_data_string_index++;
            }
            free(betaflight_data_string);

            if(got_gps){
                char* betaflight_gps_string = betaflight_blackbox_get_encoded_gps_string(
                    bn357_get_utc_time_raw(),
                    bn357_get_satellites_quantity(),
                    bn357_get_latitude_decimal_format(),
                    bn357_get_longitude_decimal_format(),
                    bn357_get_altitude_meters(),
                    0,
                    0,
                    &data_size // it will append but not overwrite
                );

                uint16_t betaflight_gps_string_index = 0;
                while(sd_card_buffer_index < data_size){
                    sd_card_buffer[sd_card_buffer_index] = betaflight_gps_string[betaflight_gps_string_index];
                    sd_card_buffer_increment_index();
                    sd_card_buffer_index++;
                    betaflight_gps_string_index++;
                }
                free(betaflight_gps_string);
            }
        }else{
            // Log a bit of data
            // sd_card_append_to_buffer(1, "%02d:%02d:%02d:%03d;", time_since_startup_hours, time_since_startup_minutes, time_since_startup_seconds, time_since_startup_ms);
            // sd_card_append_to_buffer(1, "ACCEL,%.2f,%.2f,%.2f;", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
            // sd_card_append_to_buffer(1, "GYRO,%.2f,%.2f,%.2f;", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);
            // sd_card_append_to_buffer(1, "MAG,%.2f,%.2f,%.2f;", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);
            // sd_card_append_to_buffer(1, "MOTOR,1=%.2f,2=%.2f,3=%.2f,4=%.2f;", motor_power[0], motor_power[1], motor_power[2], motor_power[3]);
            // sd_card_append_to_buffer(1, "ERROR,pitch=%.2f,roll=%.2f,yaw=%.2f,altitude=%.2f;", error_pitch, error_roll, error_yaw, error_altitude);
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
                    sd_special_write_chunk_of_byte_data_async(sd_card_get_buffer_pointer(1), data_size);
                    sd_buffer_swap();
                    sd_buffer_clear(1);
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

    // Update the blue led with current sd state
    if(sd_card_initialized){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }else{
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }
}


void handle_loop_end(){
    loop_iteration++;
    handle_loop_timing();
}

void init_STM32_peripherals(){
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);

    HAL_Delay(1);
    MX_DMA_Init(); // This has to be before the uart inits, otherwise dma interrupts dont work
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_SPI3_Init();
    HAL_Delay(1);
    MX_USART2_UART_Init();
    MX_USART1_UART_Init();
    RetargetInit(&huart1);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void calibrate_escs(){
    // Delay for things to settle down in the mcu init
    HAL_Delay(2000);

    // Set the max value
    uint16_t max_pwm = setServoActivationPercent(100, min_esc_pwm_value, max_esc_pwm_value);
    printf("Max: %d\n", max_pwm);
    TIM1->CCR1 = max_pwm;
    TIM1->CCR4 = max_pwm;
    TIM2->CCR1 = max_pwm;
    TIM2->CCR2 = max_pwm;
    HAL_Delay(3000);
    
    // Set the min value
    uint16_t min_pwm = setServoActivationPercent(0, min_esc_pwm_value, max_esc_pwm_value);
    printf("Min: %d\n", min_pwm);
    TIM1->CCR1 = min_pwm;
    TIM1->CCR4 = min_pwm;
    TIM2->CCR1 = min_pwm;
    TIM2->CCR2 = min_pwm;

    // 10 Seconds of min value init just to make sure
    HAL_Delay(6000);

    // After calibration remember that the 1 percent throttle might not do anything and it only starts moving at 3 percent throttle
    // This is if it is starting from 0
}

uint8_t init_sensors(){
    printf("-----------------------------INITIALIZING MODULES...\n");

    uint8_t mpu6050 = init_mpu6050(
        &hi2c1, 
        ACCEL_CONFIG_RANGE_2G, 
        GYRO_CONFIG_RANGE_250_DEG, 
        LOW_PASS_FILTER_FREQUENCY_CUTOFF_GYRO_21HZ_ACCEL_20HZ,
        1, 
        accelerometer_scale_factor_correction, 
        accelerometer_correction, 
        gyro_correction, 
        REFRESH_RATE_HZ, 
        complementary_ratio, 
        complementary_beta
    );

    uint8_t mmc5603 = mmc5603_init(&hi2c1, 1, hard_iron_correction, soft_iron_correction, 1, 200, 1);
    uint8_t bmp280 = init_bmp280(&hi2c1);
    uint8_t bn357 = init_bn357(&huart2, &hdma_usart2_rx, 0);
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
    // bn357_start_uart_interrupt();

    return 1;
}

void init_loop_timer(){
    loop_start_time = HAL_GetTick();
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
    mpu6050_get_accelerometer_readings_gravity(acceleration_data);
    mpu6050_get_gyro_readings_dps(gyro_angular);
    mmc5603_magnetometer_readings_micro_teslas(magnetometer_data);

    calculate_degrees_x_y(acceleration_data, &accelerometer_x_rotation, &accelerometer_y_rotation, accelerometer_x_roll_offset, accelerometer_y_pitch_offset);
    // Get a good raw yaw for calculations later
    calculate_yaw(magnetometer_data, &magnetometer_z_rotation);
    last_raw_yaw = magnetometer_z_rotation;

    calculate_yaw_tilt_compensated(magnetometer_data, &magnetometer_z_rotation, accelerometer_x_rotation, accelerometer_y_rotation);

    gyro_degrees[0] = accelerometer_x_rotation;
    gyro_degrees[1] = accelerometer_y_rotation;
    gyro_degrees[2] = magnetometer_z_rotation;
    printf("Initial location acceleration: %f, %f, %f\n", acceleration_data[0], acceleration_data[1], acceleration_data[2]);

    printf("Initial location x: %.2f y: %.2f, z: %.2f\n", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);

    // Set the desired yaw as the initial one
    target_yaw = gyro_degrees[2];

    // bmp280_set_reference_pressure_from_number_of_samples(15);
}

void handle_loop_timing(){
    loop_end_time = HAL_GetTick();
    delta_loop_time = loop_end_time - loop_start_time;

    // printf("b%d", delta_loop_time);
    
    // This one is more precise than using HAL_Delay
    while((1000 / REFRESH_RATE_HZ) > HAL_GetTick()-loop_start_time);

    uint32_t temp_loop_start_time = loop_start_time;
    loop_start_time = HAL_GetTick();

    loop_end_time = HAL_GetTick();
    delta_loop_time = loop_end_time - temp_loop_start_time;
    // printf("a%d\n", delta_loop_time);
}

// 0.5ms to 2ms = range is 1.5ms
// 0.5 is 2.5%  and is 25 in the float value passed to this
// 2   is 10%   amd is 100 in the float value passed to this
// 100 - 25 = 75

// default min and max is 25 and 75
uint16_t setServoActivationPercent(float percent, uint16_t minValue, uint16_t maxValue)
{
    if(percent > 100.0){
        percent = 1.0;
    }else if(percent < 0.0){
        percent = 0.0;
    }else{
        percent = percent / 100.0;
    }
    return percent * (maxValue - minValue) + minValue;
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
    //printf("'%s'\n", type_output);
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

// Print out how much time has passed since the start of the loop. To debug issues with performance
void track_time(){
    uint32_t delta_loop_time_temp = loop_end_time - loop_start_time;

    printf("%5ld ms ", delta_loop_time_temp);
}

// Map value from a specified range to a new range
float map_value(float value, float input_min, float input_max, float output_min, float output_max) {
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
        BASE_PITCH_ROLL_GAIN_P, 
        BASE_PITCH_ROLL_GAIN_I, 
        BASE_PITCH_ROLL_GAIN_D,
        BASE_PITCH_ROLL_MASTER_GAIN
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
        added_pitch_roll_gain_p, 
        added_pitch_roll_gain_i, 
        added_pitch_roll_gain_d,
        added_pitch_roll_master_gain
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
    for (uint8_t i = 0; i < array_size; i++){
        array[i] = 0.0f;
    }
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
  RCC_OscInitStruct.PLL.PLLN = 75;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19997-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19997-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.BaudRate = 9600;
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
