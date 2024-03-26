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

// Drivers
#include "../lib/mpu6050/mpu6050.h"
#include "../lib/qmc5883l/qmc5883l.h"
#include "../lib/bmp280/bmp280.h"
#include "../lib/bme280/bme280.h"
#include "../lib/bmp680/bmp680.h"
#include "../lib/mpl3115a2/mpl3115a2.h"
#include "../lib/ms5611/ms5611.h"
#include "../lib/bn357/bn357.h"
#include "../lib/nrf24l01/nrf24l01.h"
// #include "../lib/sd_card/sd_card.h"
#include "../lib/sd_card/sd_card_spi.h"


// Other imports
#include "../lib/utils/ned_coordinates/ned_coordinates.h"
#include "../lib/pid/pid.h"

void init_STM32_peripherals();
void calibrate_escs();
void fix_mag_axis(float *magnetometer_data);
void fix_gyro_axis(float *accelerometer_data_temp);
uint16_t setServoActivationPercent(float percent, uint16_t minValue, uint16_t maxValue);
double mapValue(double value, double input_min, double input_max, double output_min, double output_max);
void extract_request_values(char *request, uint8_t request_size, uint8_t *throttle, uint8_t *yaw, uint8_t *pitch, uint8_t *roll);
double get_sensor_fusion_altitude(double gps_altitude, double barometer_altitude);
u_int8_t init_sensors();
void init_loop_timer();
void check_calibrations();
void calibrate_gyro();
void get_initial_position();
void handle_loop_timing();

void extract_joystick_request_values_uint(char *request, uint8_t request_size, uint8_t *throttle, uint8_t *yaw, uint8_t *roll, uint8_t *pitch);
void extract_joystick_request_values_float(char *request, uint8_t request_size, float *throttle, float *yaw, float *roll, float *pitch);
void extract_request_type(char *request, uint8_t request_size, char *type_output);
void extract_pid_request_values(char *request, uint8_t request_size, double *added_proportional, double *added_integral, double *added_derivative, double *added_master);
void track_time();
double map_value(double value, double input_min, double input_max, double output_min, double output_max);
double apply_dead_zone(double value, double max_value, double min_value, double dead_zone);
void send_pid_base_info_to_remote();
void send_pid_added_info_to_remote();
char* generate_message_pid_values_nrf24(double base_proportional, double base_integral, double base_derivative, double base_master);

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

// SPI pins
// PA5  SPI1 SCK
// PA6  SPI1 MISO
// PA7  SPI1 MOSI

// GPIO
// PC13 Internal LED
// PB5  SPI RADIO
// PB4  SPI RADIO
// PA12 LED


/**
 * Timer peripheral clock: 50000000Hz
 * Pre scaler value: 250
 * Counter period: 4000
 * Timer clock divided by pre scaler: 200000.0Hz
 * Frequency: 50.0Hz
 * Max duty cycle: 20.0ms
 * Pwm value range: 0-4000
 * Resolution per pwm step: 0.005ms
 *

 * 
 * The esc is shit so we have different max and min than normal
 * 1.08ms/20ms * 4000(Counter period) = 216
 * 1.94ms/20ms * 4000 = 388
 */
const uint16_t max_esc_pwm_value = 388;
const uint16_t actual_max_esc_pwm_value = 370; // (max lipo amp rating / max draw of a bldc motor being used x 4) 0.917 * (max_pwm - min_pwm) + min_pwm = 371.4
const uint16_t min_esc_pwm_value = 188;

#define GPS_OUTPUT_BUFFER_SIZE 550
#define GPS_RECEIVE_BUFFER_SIZE 550

uint8_t gps_output_buffer[GPS_OUTPUT_BUFFER_SIZE];
uint8_t receive_buffer[GPS_RECEIVE_BUFFER_SIZE];

// Interrupt for uart 2 data received
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // make sure the dma is initialized before uart for this to work
    // and that dma is initialized after GPIO.

    // If the stm32 is restarted  while it is initializing dma the
    // data arrives from the uart and it fucks up. So you have to restart it a few times

    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

    if (huart->Instance == USART2)
    {
        // printf("Interrupt UART 2\n");
        memcpy((uint8_t *)gps_output_buffer, receive_buffer, Size);
        // unsigned char gpsString[] = "dfdgfhdfdfhsdfh$GNGGA,,,N,,E,,,,,M,,M,,*7413.786016\ndasdasd";
        // bn357_parse_and_store(gpsString, Size);
        bn357_parse_and_store((unsigned char*)gps_output_buffer, Size);
        /* start the DMA again */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)receive_buffer, GPS_RECEIVE_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}

// Interrupt for uart 2 when it crashes to restart it
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // printf("Error UART 2\n");
        HAL_UART_DeInit(&huart2);
        HAL_UART_Init(&huart2);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receive_buffer, GPS_RECEIVE_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}


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

// float hard_iron_correction[3] = {
//     0, 0, 0
// };

// float soft_iron_correction[3][3] = {
//     {1,0,0},
//     {0,1,0},
//     {0,0,1}
// };


float hard_iron_correction[3] = {
    -26.418969, 308.296635, 189.949606
};

float soft_iron_correction[3][3] = {
    {3.650759,0.250193,-0.058046},
    {0.250193,4.132441,-0.245339},
    {-0.058046,-0.245339,3.733419}
};

float accelerometer_correction[3] = {
    0.024980, -0.020180, 1.144808
};
float gyro_correction[3] = {
    -2.651069, 3.157305, 0.638099
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
double error_pitch = 0;
double error_roll = 0;
double error_yaw = 0;
double error_altitude = 0;

// Actual PID adjustment for pitch
// const double base_pitch_master_gain = 1.0;
// const double base_pitch_gain_p = 0.0; // 0.35
// const double base_pitch_gain_i = 0.0; // 0.0
// const double base_pitch_gain_d = 0.0; // 130.0
#define BASE_PITCH_ROLL_MASTER_GAIN 0.4
#define BASE_PITCH_ROLL_GAIN_P 0.0
#define BASE_PITCH_ROLL_GAIN_I 0.0
#define BASE_PITCH_ROLL_GAIN_D 0.0

// Before idle motors
// const double gain_p = 0.41; 
// const double gain_i = 0.1;
// const double gain_d = 0.0;

// Notes for PID
// 1) It is important that the natural state without power of the drone is not stable, otherwise 
// some offset of center off mass or pid desired point needs to introduced
// 2) Remember that the drone when in the air has motors spinning at idle power, just enough 
// to float in the air. If you are testing with drone constrained add a base motor speed to account for this.


// With quadcopter got 1.4 on P and it balanced for a bit but still wobbly

// PID for yaw
const float yaw_gain_p = 0.0; 
const float yaw_gain_i = 0.0;
const float yaw_gain_d = 0.0;

// PID for yaw
const float altitude_gain_p = 0.0; 
const float altitude_gain_i = 0.0;
const float altitude_gain_d = 0.0;

// Used for smooth changes to PID while using remote control. Do not touch this

double pitch_roll_master_gain = BASE_PITCH_ROLL_MASTER_GAIN; // Dont you just love the STM#2 compiler?
double pitch_roll_gain_p = BASE_PITCH_ROLL_GAIN_P;
double pitch_roll_gain_i = BASE_PITCH_ROLL_GAIN_I;
double pitch_roll_gain_d = BASE_PITCH_ROLL_GAIN_D;

double added_pitch_roll_master_gain = 0;
double added_pitch_roll_gain_p = 0;
double added_pitch_roll_gain_i = 0;
double added_pitch_roll_gain_d = 0;

// Refresh rate ##############################################################################################

// remember that the stm32 is not as fast as the esp32 and it cannot print lines at the same speed
// const float refresh_rate_hz = 400;
#define REFRESH_RATE_HZ 1

// Sensor stuff ##############################################################################################
float complementary_ratio = 1.0 - 1.0/(1.0+(1.0/REFRESH_RATE_HZ)); // Depends on how often the loop runs. 1 second / (1 second + one loop time)
float acceleration_data[] = {0,0,0};
float gyro_angular[] = {0,0,0};
float gyro_degrees[] = {0,0,0};
float magnetometer_data[] = {0,0,0};
double gps_longitude = 0.0;
double gps_latitude = 0.0;
float pressure = 0.0;
float temperature = 0.0;
float altitude = 0.0;

double target_pitch = 0.0;
double target_roll = 0.0;
double target_yaw = 0.0;
double target_altitude = 0.0;

double motor_power[] = {0.0, 0.0, 0.0, 0.0};


// Remote control settings ############################################################################################
double max_yaw_attack = 20.0;
double max_pitch_attack = 10;
double pitch_attack_step = 0.1;

double max_roll_attack = 10;
double roll_attack_step = 0.1;

// uint8_t throttle = 0;
// uint8_t yaw = 50;
// uint8_t last_yaw = 50;
// uint8_t pitch = 50;
// uint8_t roll = 50;

float throttle = 0.0;
float yaw = 50.0;
float last_yaw = 50.0;
float pitch = 50.0;
float roll = 50.0;

uint8_t shit_encoder_mode = 0;
uint8_t slowing_lock = 0;

double last_raw_yaw = 0;
double delta_yaw = 0;

float minimum_signal_timing_seconds = 0.2; // Seconds
uint32_t last_signal_timestamp = 0;


// For deciding which log file to log to
uint16_t log_file_index = 1;
char log_file_base_name[] = "Quadcopter.txt";
#define LOG_FILE_NAME_MAX 45
char log_file_name[LOG_FILE_NAME_MAX];
uint8_t log_file_location_found = 0;
uint8_t sd_card_initialized = 0;
uint8_t log_loop_count = 0;

int main(void){
    init_STM32_peripherals();
    printf("STARTING PROGRAM\n"); 
    // calibrate_escs();
    if(init_sensors() == 0){
        return 0; // exit if initialization failed
    }
    // check_calibrations();
    // calibrate_gyro(); // Recalibrate the gyro as the temperature affects the calibration
    get_initial_position();

    struct pid pitch_pid = pid_init(pitch_roll_master_gain * pitch_roll_gain_p, pitch_roll_master_gain * pitch_roll_gain_i, pitch_roll_master_gain * pitch_roll_gain_d, 0.0, HAL_GetTick(), 20.0, -20.0, 1);
    struct pid roll_pid = pid_init(pitch_roll_master_gain * pitch_roll_gain_p, pitch_roll_master_gain * pitch_roll_gain_i, pitch_roll_master_gain * pitch_roll_gain_d, 0.0, HAL_GetTick(), 20.0, -20.0, 1);
    struct pid yaw_pid = pid_init(yaw_gain_p, yaw_gain_i, yaw_gain_d, 0.0, HAL_GetTick(), 0, 0, 0);
    struct pid altitude_pid = pid_init(altitude_gain_p, altitude_gain_i, altitude_gain_d, 0.0, HAL_GetTick(), 0, 0, 0);



    sd_card_initialized = 1;


    #define SPI_BIT_BANG_TRANSMIT_BUFFER_SIZE 100
    uint8_t transmit_buffer[SPI_BIT_BANG_TRANSMIT_BUFFER_SIZE];
    uint8_t transmit_buffer_index = 0;

transmit_buffer[0] = 0b01010101;

    uint8_t bit_index_counter = 0;
    while(1){
        uint8_t MISO_state = (transmit_buffer[transmit_buffer_index] >> (7 - bit_index_counter)) & 0x01;

        printf("Number %d\n", MISO_state);

        bit_index_counter++;
        bit_index_counter = bit_index_counter % 8;

        if(bit_index_counter == 0){
            break;
        }
    }


    uint8_t sd_status = 0;
    sd_status = sd_card_initialize_spi(&hspi3, GPIOA, GPIO_PIN_15, GPIOA, GPIO_PIN_12);
    if(!sd_test_interface()){
        printf("FAILED SD INTERFACE TEST\n");
        while (1);
    }
    sd_status = sd_card_initialize();
    sd_status = sd_open_file("Quadcopter.txt", FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
    sd_status = sd_close_file();
    sd_status = sd_card_deinitialize();

    sd_status = sd_card_initialize();
    sd_status = sd_open_file("Quadcopter.txt", FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
    sd_status = sd_set_file_cursor_offset(sd_card_get_selected_file_size());
    sd_status = sd_write_data_to_file("Quadcopter restarted\n");
    sd_status = sd_write_data_to_file("Quadcopter data2\n");
    sd_status = sd_write_data_to_file("Quadcopter data3\n");
    sd_status = sd_write_data_to_file("Quadcopter data4\n");
    sd_status = sd_close_file();
    sd_status = sd_open_file("Quadcopter.txt", FA_READ);
    sd_status = sd_read_data_from_file();
    char* data = sd_card_get_buffer_pointer();
    if(data){
        sd_status = printf("Read Data : %s\n", data);
    }
    sd_buffer_clear();
    sd_status = sd_close_file();
    sd_status = sd_open_file("Quadcopter.txt", FA_WRITE);
    sd_status = sd_set_file_cursor_offset(sd_card_get_selected_file_size()); // set cursor to end of file to so data is not overwritten
    sd_status = sd_write_data_to_file("Quadcopter restarted\n");
    sd_status = sd_write_data_to_file("Quadcopter data2\n");
    sd_status = sd_write_data_to_file("Quadcopter data3\n");
    sd_status = sd_write_data_to_file("Quadcopter data4\n");
    sd_status = sd_close_file();
    sd_status = sd_open_file("Quadcopter.txt", FA_READ);
    sd_status = sd_read_data_from_file();
    char* data1 = sd_card_get_buffer_pointer();
    if(data1){
        sd_status = printf("Read Data : %s\n", data1);
        free(data1);
    }

    sd_buffer_clear();
    sd_status = sd_close_file();
    sd_status = sd_card_deinitialize();

    while (1)
    {
        /* code */
    }
    
    // uint8_t command = 0x00000001;

    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    // printf("Looping\n");
    // while(1){
        // printf("Loop\n");
        // HAL_Delay(2000);
        // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

        // if(command == 1){
        //     command = 0;
        // }else{
        //     command = 1;
        // }
        // HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi3, &command, 1, 10);
        // uint8_t response[1] = {0};
        // wait_for_slave_ready();
        // HAL_SPI_Receive(&hspi3, response, 1, 10);

        
        // if(response[0] == 1 && command == 1){
        //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        // }else if(response[0] == 1 && command == 0){
        //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        // }else{
        //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        // }

        // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    // }

    // Try to initialize the file and test some 

    sd_card_initialized = sd_card_initialize();
    if(sd_card_initialized){
        printf("Looking for viable log file.\n");
        do{
            uint16_t log_file_name_length = sprintf(log_file_name, "%d%s", log_file_index, log_file_base_name);

            // Quit of the string is too big
            if(log_file_name_length > LOG_FILE_NAME_MAX){
                printf("Log file string too long\n");
                return 0;
            }
            printf("Looking for viable log file. Testing file name: %s\n", log_file_name);
            log_file_location_found = !sd_file_exists(log_file_name);
            log_file_index++;
        }
        while(!log_file_location_found);
        printf("Found viable log file.\n");
        sd_open_file(log_file_name, FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
        sd_close_file();
        sd_card_initialized = sd_open_file(log_file_name, FA_WRITE);
        sd_set_file_cursor_offset(sd_card_get_selected_file_size());
    }

    printf("Looping\n");
    init_loop_timer();
    while (1){
        // printf("Loop ");
        
        // Gathering data from sensors and transforming it: 2 ms at most
        // Read sensor data ######################################################################################################################
        // gps_latitude = bn357_get_latitude_decimal_format();
        // gps_longitude = bn357_get_longitude_decimal_format();
        // temperature = bmp280_get_temperature_celsius();

        // calculate the altitude using gps altitude and a bmp280 reference altitude
        // Reset the reference on bmp280 every time the gps gets updated
        // So the barometer keeps track in between the gps updates and does so with the 
        // origin of the precise altitude value from gps

        // altitude = get_sensor_fusion_altitude(bn357_get_altitude_meters() ,(double)bmp280_get_height_meters_from_reference(bn357_get_status_up_to_date(1)));


        mpu6050_get_accelerometer_readings_gravity(acceleration_data);
        mpu6050_get_gyro_readings_dps(gyro_angular);
        qmc5883l_magnetometer_readings_micro_teslas(magnetometer_data);

        // Convert the sensor data to data that is useful
        fix_mag_axis(magnetometer_data); // Switches around the x and the y of the magnetometer to match mpu6050 outputs
        calculate_degrees_x_y(acceleration_data, &accelerometer_x_rotation, &accelerometer_y_rotation); // Get roll and pitch from the data. I call it x and y. Ranges -90 to 90. 

        // My mpu6050 has a drift problem when rotating around z axis. I have only seen Joop Broking having this problem.
        // Raw yaw - yaw without tilt adjustment
        // Yaw - yaw with tilt adjustment
        // Get raw yaw. The results of this adjustment have to modify the values of gyro degrees before complementary filter. So only the non tilt adjusted yaw is available
        calculate_yaw(magnetometer_data, &magnetometer_z_rotation);

        // if(yaw != 50){ // This is not an issue when not rotating.
        //     // Joop Brokings method did not work for me and it didn't make sense subtracting scaled total yaw. I subtract delta yaw instead
        //     delta_yaw = angle_difference(magnetometer_z_rotation, last_raw_yaw); // Helper function to handle wrapping

        //     // gyro_angular[0] -= delta_yaw * -16.5; 
        //     gyro_angular[0] -= delta_yaw * -5.0; 

        //     // This robot doesn't use the y axis so i ignore it 
        // }

        // Save the raw yaw for next loop. No mater if yaw changes or not. Need to know the latest one
        last_raw_yaw = magnetometer_z_rotation;

        // Use complementary filter to correct the gyro drift. 
        convert_angular_rotation_to_degrees_x_y(gyro_angular, gyro_degrees, accelerometer_x_rotation, accelerometer_y_rotation, HAL_GetTick(), 1);

        // Get yaw that is adjusted by x and y degrees
        calculate_yaw_tilt_compensated(magnetometer_data, &magnetometer_z_rotation, gyro_degrees[0], gyro_degrees[1]);

        // Complementary filter did not work good for magnetometer+gyro. 
        // Gyro just too slow and inaccurate for this.
        // I trust the magnetometer more than the gyro in this case.
        gyro_degrees[2] = magnetometer_z_rotation;

        fix_gyro_axis(gyro_degrees); // switch the x and y axis of gyro

        // Receive remote control data ###########################################################################################################
        if(nrf24_data_available(1)){ // takes 3-4 ms
            nrf24_receive(rx_data); // takes 8-9 ms
            // Get the type of request
            extract_request_type(rx_data, strlen(rx_data), rx_type);

            if(strcmp(rx_type, "js") == 0){
                last_signal_timestamp = HAL_GetTick();

                // extract_joystick_request_values_uint(rx_data, strlen(rx_data), &throttle, &yaw, &roll, &pitch);
                extract_joystick_request_values_float(rx_data, strlen(rx_data), &throttle, &yaw, &roll, &pitch);


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
                    if(shit_encoder_mode){
                        // Because my encoder is shit it prefers to have the extremes of pitch as 
                        // that doesn't overwhelm the pid of it.
                        if(pitch > 60){
                            target_pitch = target_pitch + pitch_attack_step;
                        }else if(pitch < 40){
                            target_pitch = target_pitch - pitch_attack_step;
                        }
                    }else{
                        target_pitch = target_pitch + map_value(pitch, 0.0, 100.0, -pitch_attack_step, pitch_attack_step);
                    }
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
                    if(shit_encoder_mode){
                        // Because my encoder is shit it prefers to have the extremes of roll as 
                        // that doesn't overwhelm the pid of it.
                        if(roll > 60){
                            target_roll = target_roll + roll_attack_step;
                        }else if(pitch < 40){
                            target_roll = target_roll - roll_attack_step;
                        }
                    }else{
                        target_roll = target_roll + map_value(roll, 0.0, 100.0, -roll_attack_step, roll_attack_step);
                    }
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
                    target_yaw = gyro_degrees[2] + map_value(yaw, 0.0, 100.0, -max_yaw_attack, max_yaw_attack);
                    // handle the switch from -180 to 180 degrees
                    if(target_yaw > 180.0){
                        target_yaw = target_yaw - 360.0;
                    }else if(target_yaw < -180.0){
                        target_yaw = target_yaw + 360.0;
                    }
                }

                // Reset the yaw to the current degrees
                if(last_yaw != 50 && yaw == 50){
                    target_yaw = gyro_degrees[2];
                }

                last_yaw = yaw;

            }else if(strcmp(rx_type, "pid") == 0){
                printf("\nGot pid");

                double added_proportional = 0;
                double added_integral = 0;
                double added_derivative = 0;
                double added_master_gain = 0;

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
            }

            rx_type[0] = '\0'; // Clear out the string by setting its first char to string terminator
        }

        // double error_altitude = mapValue(pid_get_error(&altitude_pid, altitude, HAL_GetTick()), -180.0, 180.0, -100.0, 100.0);
        // printf("Altitude error %6.2f ", error_altitude);
        
        // For the robot to do work it needs to be receiving radio signals and at the correct angles, facing up
        if(
            gyro_degrees[0] <  30 && 
            gyro_degrees[0] > -30 && 
            gyro_degrees[1] <  30 && 
            gyro_degrees[1] > -30 && 
            ((double)HAL_GetTick() - (double)last_signal_timestamp) / 1000.0 <= minimum_signal_timing_seconds
        ){
            // pid_set_desired_value(&pitch_pid, target_pitch);
            // pid_set_desired_value(&roll_pid, target_roll);
            // pid_set_desired_value(&yaw_pid, target_yaw);
            // pid_set_desired_value(&altitude_pid, target_altitude);

            // pitch is facing to the sides
            // roll is facing forwards and backwards
            error_pitch = pid_get_error(&pitch_pid, gyro_degrees[0], HAL_GetTick());
            error_roll = pid_get_error(&roll_pid, gyro_degrees[1], HAL_GetTick());
            // error_yaw = pid_get_error(&roll_pid, gyro_degrees[2], HAL_GetTick());
            // error_altitude = pid_get_error(&roll_pid, altitude, HAL_GetTick());

            error_altitude = throttle*0.9;

            motor_power[0] = error_altitude + (-error_pitch) +  (-error_roll);
            motor_power[1] = error_altitude + (-error_pitch) +  ( error_roll);
            motor_power[2] = error_altitude + ( error_pitch) +  ( error_roll);
            motor_power[3] = error_altitude + ( error_pitch) +  (-error_roll);


            // Motor A (4) 13740 rpm or 229 rotations per second
            // Motor B (1) 14460 rpm or 241
            // Motor C (2) 14160 rpm or 236
            // Motor D (3) 14460 rpm or 241


            // GPS side
            TIM2->CCR1 = setServoActivationPercent(motor_power[2], min_esc_pwm_value, actual_max_esc_pwm_value);
            TIM2->CCR2 = setServoActivationPercent(motor_power[3], min_esc_pwm_value, actual_max_esc_pwm_value);

            // No gps side
            TIM1->CCR1 = setServoActivationPercent(motor_power[0], min_esc_pwm_value, actual_max_esc_pwm_value);
            TIM1->CCR4 = setServoActivationPercent(motor_power[1], min_esc_pwm_value, actual_max_esc_pwm_value);
            // printf("1 %f\n", throttle);
        }else{
            // GPS side
            TIM2->CCR1 = setServoActivationPercent(0, min_esc_pwm_value, actual_max_esc_pwm_value);
            TIM2->CCR2 = setServoActivationPercent(0, min_esc_pwm_value, actual_max_esc_pwm_value);

            // No gps side
            TIM1->CCR1 = setServoActivationPercent(0, min_esc_pwm_value, actual_max_esc_pwm_value);
            TIM1->CCR4 = setServoActivationPercent(0, min_esc_pwm_value, actual_max_esc_pwm_value);
            // printf("0\n");
        }

        // Print out for debugging
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


        if(sd_card_initialized){
            // sd_card_append_to_buffer("ACCEL, %6.2f, %6.2f, %6.2f, ", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
            sd_card_append_to_buffer("GYRO, %6.2f, %6.2f, %6.2f, ", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);
            // sd_card_append_to_buffer("MAG, %6.2f, %6.2f, %6.2f, ", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);
            // sd_card_append_to_buffer("TEMP %6.5f, ", temperature);
            // sd_card_append_to_buffer("ALT %6.2f, ", altitude);
            // sd_card_append_to_buffer("GPS %f, %f, ", gps_longitude, gps_latitude);
            // sd_card_append_to_buffer("ACCEL, %6.2f, %6.2f, %6.2f, ", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
            sd_card_append_to_buffer("\n");
            
            // printf(" Logged data to SD buffer. ");
            log_loop_count++;
        }


        if(sd_card_initialized && log_loop_count > 4){
            // sd_card_initialize();

            // sd_close_file();
            // sd_card_initialized = sd_open_file(log_file_name, FA_WRITE);
            // sd_set_file_cursor_offset(sd_card_get_selected_file_size());
            sd_card_initialized = sd_write_buffer_to_file();
            sd_save_file();
            // sd_close_file();
            sd_buffer_clear();
            log_loop_count = 0;

            // sd_card_deinitialize();
            
            // printf(" Logged data from buffer to SD. ");
        }

        // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

        // if(command == 1){
        //     command = 0;
        // }else{
        //     command = 1;
        // }
        // HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi3, &command, 1, 10);
        // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

        // printf("Sent SPI3: %d\n", result);

        
        // printf("\n");

        fix_gyro_axis(gyro_degrees); // switch back the x and y axis of gyro to how they were before. This is for sensor fusion not to be confused 

        handle_loop_timing();
    }
}

void init_STM32_peripherals(){
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);

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
    // MX_FATFS_Init();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    // HAL_UART_RegisterCallback(&huart2, HAL_UART_ERROR_CB_ID, HAL_UART_ErrorCallback);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receive_buffer, GPS_RECEIVE_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
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
    HAL_Delay(10000);

    // After calibration remember that the 1 percent throttle might not do anything and it only starts moving at 3 percent throttle
    // This is if it is starting from 0
}

uint8_t init_sensors(){
    printf("-----------------------------INITIALIZING MODULES...\n");

    uint8_t mpu6050 = init_mpu6050(&hi2c1, 1, accelerometer_correction, gyro_correction, REFRESH_RATE_HZ, complementary_ratio);
    uint8_t qmc5883l = init_qmc5883l(&hi2c1, 1, hard_iron_correction, soft_iron_correction);

    uint8_t bmp280 = init_bmp280(&hi2c1);
    // uint8_t bme280 = init_bme280(&hi2c1);
    // uint8_t bmp680 = init_bmp680(&hi2c1);
    // uint8_t ms5611 = init_ms5611(&hi2c1);
    // uint8_t mpl3115a2 = init_mpl3115a2(&hi2c1);

    uint8_t bn357 = init_bn357(&huart2, 0);
    uint8_t nrf24 = init_nrf24(&hspi1);

    printf("-----------------------------INITIALIZING MODULES DONE... ");

    if (mpu6050 && qmc5883l && bmp280 && bn357 && nrf24){
        printf("OK\n");
    }else{
        printf("NOT OK\n");
        return 0;
    }

    // Continue initializing
    nrf24_rx_mode(tx_address, 10);

    return 1;
}

void init_loop_timer(){
    loop_start_time = HAL_GetTick();
}

void check_calibrations(){
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
    mpu6050_apply_calibrations(accelerometer_correction, gyro_correction_temp);
}



void get_initial_position(){
    // Find the initial position in degrees and apply it to the gyro measurement integral
    // This will tell the robot which way to go to get the actual upward
    mpu6050_get_accelerometer_readings_gravity(acceleration_data);
    mpu6050_get_gyro_readings_dps(gyro_angular);
    qmc5883l_magnetometer_readings_micro_teslas(magnetometer_data);
    fix_mag_axis(magnetometer_data); // Switches around the x and the y to match mpu6050

    calculate_degrees_x_y(acceleration_data, &accelerometer_x_rotation, &accelerometer_y_rotation);

    // Get a good raw yaw for calculations later
    calculate_yaw(magnetometer_data, &magnetometer_z_rotation);
    last_raw_yaw = magnetometer_z_rotation;

    calculate_yaw_tilt_compensated(magnetometer_data, &magnetometer_z_rotation, accelerometer_x_rotation, accelerometer_y_rotation);

    gyro_degrees[0] = accelerometer_x_rotation;
    gyro_degrees[1] = accelerometer_y_rotation;
    gyro_degrees[2] = magnetometer_z_rotation;

    printf("Initial location x: %.2f y: %.2f, z: %.2f\n", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);

    // Set the desired yaw as the initial one
    target_yaw = gyro_degrees[2];
}

void handle_loop_timing(){
    loop_end_time = HAL_GetTick();
    delta_loop_time = loop_end_time - loop_start_time;

    printf("Tb: %dms ", delta_loop_time);
    
    int time_to_wait = (1000 / REFRESH_RATE_HZ) - delta_loop_time;
    if (time_to_wait > 0)
    {
        HAL_Delay(time_to_wait);
    }

    loop_end_time = HAL_GetTick();
    delta_loop_time = loop_end_time - loop_start_time;
    printf("Ta: %dms\n", delta_loop_time);

    loop_start_time = HAL_GetTick();
}

// 0.5ms to 2ms = range is 1.5
// 0.5 is 2.5%  and is 25
// 2   is 10%   amd is 100
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

double mapValue(double value, double input_min, double input_max, double output_min, double output_max) {
    // Calculate the input and output ranges' lengths
    double input_range = input_max - input_min;
    double output_range = output_max - output_min;

    // Normalize the input value relative to the input range
    double normalized_value = (value - input_min) / input_range;

// Scale the normalized value according to the output range
    double scaled_value = normalized_value * output_range;

    // Add the output range's minimum value to the scaled value
    double output_value = output_min + scaled_value;

    return output_value;
}

double get_sensor_fusion_altitude(double gps_altitude, double barometer_altitude){
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

// Switch magnetometer axis. x-> y and y->x
void fix_mag_axis(float *magnetometer_data_temp)
{
    float temp = 0.0;
    temp = magnetometer_data_temp[0];
    // y is x
    magnetometer_data_temp[0] = magnetometer_data_temp[1];
    // x is -y
    magnetometer_data_temp[1] = -temp;
}


void fix_gyro_axis(float *gyro_data_temp){
    float temp = 0.0;
    temp = gyro_data_temp[0];
    // y is x
    gyro_data_temp[0] = gyro_data_temp[1];
    // x is -y
    gyro_data_temp[1] = temp;

    // gyro_data_temp[0] = -gyro_data_temp[0];
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
void extract_pid_request_values(char *request, uint8_t request_size, double *added_proportional, double *added_integral, double *added_derivative, double *added_master){
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

// Print out how much time has passed since the start of the loop. To debug issues with performance
void track_time(){
    uint32_t delta_loop_time_temp = loop_end_time - loop_start_time;

    printf("%5ld ms ", delta_loop_time_temp);
}

// Map value from a specified range to a new range
double map_value(double value, double input_min, double input_max, double output_min, double output_max) {
    // Calculate the input and output ranges' lengths
    double input_range = input_max - input_min;
    double output_range = output_max - output_min;

    // Normalize the input value relative to the input range
    double normalized_value = (value - input_min) / input_range;

    // Scale the normalized value according to the output range
    double scaled_value = normalized_value * output_range;

    // Add the output range's minimum value to the scaled value
    double output_value = output_min + scaled_value;

    return output_value;
}

// Apply a dead zone to a value 
double apply_dead_zone(double value, double max_value, double min_value, double dead_zone){
    double mid_point = (max_value + min_value) / 2;
    double half_range = (max_value - min_value) / 2;
    double normalized_value = (value - mid_point) / half_range; // this will be -1 at min_value, +1 at max_value

    double dead_zone_normalized = dead_zone / half_range;

    double return_value;

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

char* generate_message_pid_values_nrf24(double base_proportional, double base_integral, double base_derivative, double base_master){
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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  htim1.Init.Prescaler = 250-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4000-1;
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
  htim2.Init.Prescaler = 250-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4000-1;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
