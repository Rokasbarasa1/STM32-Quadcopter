/* Auto generated shit -----------------------------------------------*/
#include "main.h"

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

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
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* Actual functional code -----------------------------------------------*/

#include "../lib/printf/retarget.h"
#include "stdio.h"
#include "string.h"
#include "../lib/mpu6050/mpu6050.h"
#include "../lib/gy271_qmc5883l/gy271.h"
#include "../lib/bmp280/bmp280.h"
#include "../lib/bn357/bn357.h"
#include "../lib/utils/ned_coordinates/ned_coordinates.h"
#include "../lib/nrf24l01/nrf24l01.h"
#include "../lib/pid/pid.h"

#define RxBuf_SIZE 500
#define MainBuf_SIZE 550

/**
 * Pre-scaler: 1000
 * Counter period: 1000
 * Timer clock = 50,000,000/1000 = 50KHz
 * PWM Frequency = 50,000/1000 = 50Hz
 * Duty cycle = 1s/PWM Freq = 20ms
 *
 * For servo it needs to be from 0.5ms to 2.5ms at the most extremes.
 * These are the values to set to CCR
 * 0.5/20 * 1000(Counter period) = 25
 * 2.5/20 * 1000 = 125
 */
#define MAX_TIMER_VALUE 999

uint8_t gps_output_buffer[MainBuf_SIZE];
uint8_t receive_buffer[RxBuf_SIZE];

// Interrupt for uart 2 data received
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // make sure the dma is initialized before uart for this to work
    // and that dma is initialized after GPIO.

    // If the stm32 is restarted and while it is initializing dma the
    // data arrives from the uart it fucks up. So you have to restart it a few times

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

    if (huart->Instance == USART2)
    {
        // printf("Interrupt UART 2\n");
        memcpy((uint8_t *)gps_output_buffer, receive_buffer, Size);
        // unsigned char gpsString[] = "dfdgfhdfdfhsdfh$GNGGA,,,N,,E,,,,,M,,M,,*7413.786016\ndasdasd";
        // bn357_parse_and_store(gpsString, Size);
        bn357_parse_and_store((unsigned char*)gps_output_buffer, Size);
        /* start the DMA again */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)receive_buffer, RxBuf_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}

// Interrupt for uart 2 when it crashes to restart it
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        printf("Error UART 2\n");
        HAL_UART_DeInit(&huart2);
        HAL_UART_Init(&huart2);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receive_buffer, RxBuf_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}


// For calibrating the magnetometer I
// used a method found in MicWro Engr Video:
// https://www.youtube.com/watch?v=MinV5V1ioWg
// Mag field norm at my location is 50.503
// use software called Magneto 1.2
const float hard_iron_correction[3] = {205.801249, -329.544434, -51.579693};
const float soft_iron_correction[3][3] = {{0.178946, 0.001568, -0.007812},
                                          {0.001568, 0.181964, 0.002300},
                                          {-0.007812, 0.002300, 0.186567}};

const float accelerometer_correction[3] = {0.008026, -0.038413, 1.149903};
const float gyro_correction[3] = {-2.371043, 2.884454, 0.648193};

const float refresh_rate_hz = 50;

// gps variables
volatile double longitude = 0.0;
volatile double latitude = 0.0;


float acceleration_data[] = {0, 0, 0};
float gyro_angular[] = {0, 0, 0};
float complementary_ratio = 0.01;
float gyro_degrees[] = {0, 0, 0};
float magnetometer_data[] = {0, 0, 0};

// bmp280 data
float pressure = 0.0;
float temperature = 0.0;
float altitude = 0.0;

float north_direction[] = {0, 0, 0};
float east_direction[] = {0, 0, 0};
float down_direction[] = {0, 0, 0};

float accelerometer_x_rotation = 0;
float accelerometer_y_rotation = 0;
float accelerometer_z_rotation = 0;

// PID gains
const double gain_p = 1; 
const double gain_i = 0.01;
const double gain_d = 0.1;

double motor_power[] = {0.0, 0.0, 0.0, 0.0};

// for nrf24 radio transmissions
uint8_t tx_address[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
uint8_t rx_data[32];

// handling loop timing
uint32_t loop_start_time = 0;
uint32_t loop_end_time = 0;
int16_t deltaLoopTime = 0;

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

void init_STM32_peripherals();
void fix_mag_axis(float *magnetometer_data);
uint16_t setServoActivationPercent(float percent, uint16_t minValue, uint16_t maxValue);
double mapValue(double value, double input_min, double input_max, double output_min, double output_max);
void extract_request_values(char *request, uint8_t request_size, uint8_t *x, uint8_t *y);
double get_sensor_fusion_altitude(double gps_altitude, double barometer_altitude);
void init_sensors();
void init_loop_timer();
void check_calibrations();
void get_initial_position();
void handle_loop_timing();
void convert_angular_rotation_to_degrees();
void output_magnetometer_measurement();
void control_esc(uint8_t x, uint8_t y);

int main(void)
{
    init_STM32_peripherals();
    init_sensors();
    init_loop_timer();
    // check_calibrations();
    get_initial_position();

    // uint16_t new_value1 = setServoActivationPercent(100, 50, 150);

    // TIM1->CCR1 = new_value1;
    // TIM1->CCR4 = new_value1;
    // TIM2->CCR1 = new_value1;
    // TIM2->CCR2 = new_value1;

    // HAL_Delay(5000);
    // uint16_t new_value2 = setServoActivationPercent(0, 50, 150);

    // TIM1->CCR1 = new_value2;
    // TIM1->CCR4 = new_value2;
    // TIM2->CCR1 = new_value2;
    // TIM2->CCR2 = new_value2;

    // HAL_Delay(5000);

    uint8_t x = 50;
    uint8_t y = 50;

    uint8_t x_previous = 50;
    uint8_t y_previous = 50;

    // struct pid x_axis_pid = pid_init(gain_p, gain_i, gain_p, 0.0, HAL_GetTick(), 1.0, -1.0);
    // struct pid y_axis_pid = pid_init(gain_p, gain_i, gain_p, 0.0, HAL_GetTick(), 1.0, -1.0);

    struct pid x_pid = pid_init(gain_p, gain_i, gain_p, 0.0, HAL_GetTick(), 180.0, -180.0);
    struct pid y_pid = pid_init(gain_p, gain_i, gain_p, 0.0, HAL_GetTick(), 180.0, -180.0);

    while (1)
    {
        // for measuring time 
        // uint32_t start_point = HAL_GetTick();
        // uint32_t end_point = HAL_GetTick();
        // printf("Radio time: %d ms\n", end_point - start_point);
        // printf("New loop\n");

        // Read sensor data
        latitude = bn357_get_latitude_decimal_format();
        longitude = bn357_get_longitude_decimal_format();
        temperature = bmp280_get_temperature_celsius();

        // calculate the altitude using gps altitude and a bmp280 reference altitude
        // Reset the reference on bmp280 every time the gps gets updated
        // So the barometer keeps track in between the gps updates and does so with the 
        // origin of the precise altitude value from gps

        altitude = get_sensor_fusion_altitude(bn357_get_altitude_meters() ,(double)bmp280_get_height_meters_from_reference(bn357_get_status_up_to_date(1)));
        mpu6050_get_accelerometer_readings_gravity(acceleration_data);
        mpu6050_get_gyro_readings_dps(gyro_angular);
        gy271_magnetometer_readings_micro_teslas(magnetometer_data);
        fix_mag_axis(magnetometer_data); // Switches around the x and the y to match mpu6050


        // Calculate using sensor data
        // get_ned_coordinates(acceleration_data, magnetometer_data, north_direction, east_direction, down_direction);
        // calculate_pitch_and_roll(acceleration_data, &roll, &pitch);

        calculate_degrees_x_y(acceleration_data, &accelerometer_x_rotation, &accelerometer_y_rotation);
        calculate_yaw(magnetometer_data, &accelerometer_z_rotation);
        convert_angular_rotation_to_degrees();

        
        // // Joystick values, 50 is basically zero position

        // x = 50;
        // y = 50;
        // if(nrf24_data_available(1)){
        //     nrf24_receive(rx_data);
        //     extract_request_values((char*) rx_data, strlen((char*) rx_data), &x, &y);
        //     printf("Dat %d %d\n", x, y);
        //     x_previous = x;
        //     y_previous = y;
        // }
        // else{
        //     x = x_previous;
        //     y = y_previous;
        // }

        double error_altitude = 0.0;

        double error_x = mapValue(pid_get_error(&x_pid, gyro_degrees[0], HAL_GetTick()), -180.0, 180.0, -100.0, 100.0) ;
        double error_y = mapValue(pid_get_error(&y_pid, gyro_degrees[1], HAL_GetTick()), -180.0, 180.0, -100.0, 100.0) ;

        // mapValue(error_x)

        // y is facing forwards and backwards

        // x is facing to the sides

        motor_power[0] = error_altitude + (-error_y) + (-error_x);
        motor_power[1] = error_altitude + (-error_y) +  (error_x);
        motor_power[2] = error_altitude +  (error_y) +  (error_x);
        motor_power[3] = error_altitude +  (error_y) + (-error_x);

        // GPS side
        TIM2->CCR1 = 2 * setServoActivationPercent(motor_power[2], 25, 125);
        TIM2->CCR2 = 2 * setServoActivationPercent(motor_power[3], 25, 125);

        // No gps side
        TIM1->CCR1 = 2 * setServoActivationPercent(motor_power[0], 25, 125);
        TIM1->CCR4 = 2 * setServoActivationPercent(motor_power[1], 25, 125);
        
        // control_esc(x, y);

        
        // output_magnetometer_measurement();

        // Print out for debugging
        // printf("ACCEL, %6.2f, %6.2f, %6.2f, ", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
        // printf("GYRO, %6.2f, %6.2f, %6.2f, ", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);
        // printf("AXIS, %6.2f, %6.2f, %6.2f, ", accelerometer_x_rotation, accelerometer_y_rotation, accelerometer_z_rotation);
        


        // printf("MAG, %6.2f, %6.2f, %6.2f, ", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);
        // printf("NORTH, %6.2f, %6.2f, %6.2f, ", north_direction[0], north_direction[1], north_direction[2]);
        // printf("TEMP %6.5f, ", temperature);
        // printf("ALT %6.2f, ", altitude);
        // printf("GPS %f, %f, ", longitude, latitude);
        // printf("ERROR x=%6.2f y=%6.2f", pid_get_error(&x_pid, gyro_degrees[0], HAL_GetTick()), pid_get_error(&y_pid, gyro_degrees[1], HAL_GetTick()));
        // printf("MOTOR 1=%6.2f 2=%6.2f 3=%6.2f 4=%6.2f", motor_power[0], motor_power[1], motor_power[2], motor_power[3]);
        // printf("ERROR x=%6.2f y=%6.2f ", error_x, error_y);

        printf("\n");

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

    // HAL_UART_RegisterCallback(&huart2, HAL_UART_ERROR_CB_ID, HAL_UART_ErrorCallback);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receive_buffer, RxBuf_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

void init_sensors(){
    printf("-----------------------------INITIALIZING MODULES...\n");

    uint8_t mpu6050 = init_mpu6050(&hi2c1, 1, accelerometer_correction, gyro_correction);
    uint8_t gy271 = init_gy271(&hi2c1, 1, hard_iron_correction, soft_iron_correction);
    uint8_t bmp280 = init_bmp280(&hi2c1);
    uint8_t bn357 = init_bn357(&huart2, 0);
    uint8_t nrf24 = init_nrf24(&hspi1);

    printf("-----------------------------INITIALIZING MODULES DONE... ");

    if (mpu6050 && gy271 && bmp280 && bn357 && nrf24){
        printf("OK\n");
    }else{
        printf("NOT OK\n");
    }

    // Continue initializing
    nrf24_rx_mode(tx_address, 10);
}

void init_loop_timer(){
    loop_start_time = HAL_GetTick();
}

void check_calibrations(){
    // Checking errors of mpu6050
    find_accelerometer_error(1000);
    find_gyro_error(300);
}

void get_initial_position(){
    // Find the initial position in degrees and apply it to the gyro measurement integral
    // This will tell the robot which way to go to get the actual upward
    mpu6050_get_accelerometer_readings_gravity(acceleration_data);
    mpu6050_get_gyro_readings_dps(gyro_angular);
    gy271_magnetometer_readings_micro_teslas(magnetometer_data);
    fix_mag_axis(magnetometer_data); // Switches around the x and the y to match mpu6050

    float ax,ay;
    calculate_degrees_x_y(acceleration_data, &accelerometer_x_rotation, &accelerometer_y_rotation);
    calculate_yaw(magnetometer_data, &accelerometer_z_rotation);

    // gyro_degrees[0] = ax;
    // gyro_degrees[1] = ay;
    gyro_degrees[0] = accelerometer_x_rotation;
    gyro_degrees[1] = accelerometer_y_rotation;
    gyro_degrees[2] = accelerometer_z_rotation;
    printf("Initial location x: %.2f y: %.2f, z: %.2f\n", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);
}

void handle_loop_timing(){
    loop_end_time = HAL_GetTick();
    deltaLoopTime = loop_end_time - loop_start_time;
    // printf("Tim: %d ms\n", deltaLoopTime);
    if (deltaLoopTime < (1000 / refresh_rate_hz))
    {
        HAL_Delay((1000 / refresh_rate_hz) - deltaLoopTime);
    }
    loop_start_time = HAL_GetTick();
}

void convert_angular_rotation_to_degrees(){
    // Convert angular velocity to actual degrees that it moved and add it to the integral (dead reckoning not PID)
    gyro_degrees[0] += (gyro_angular[0] * (1.0 / refresh_rate_hz));
    gyro_degrees[1] += (gyro_angular[1] * (1.0 / refresh_rate_hz));
    gyro_degrees[2] += (gyro_angular[2] * (1.0 / refresh_rate_hz));

    // Apply complimentary filter

    // printf("%.2f - %.2f = %.2f       ", gyro_degrees[0], accelerometer_x_rotation, gyro_degrees[0] - accelerometer_x_rotation);
    gyro_degrees[0] = gyro_degrees[0] * (1.0 - complementary_ratio) - (complementary_ratio * (gyro_degrees[0] - accelerometer_x_rotation));
    gyro_degrees[1] = gyro_degrees[1] * (1.0 - complementary_ratio) - (complementary_ratio * (gyro_degrees[1] - accelerometer_y_rotation));
    gyro_degrees[2] = gyro_degrees[2] * (1.0 - complementary_ratio) - (complementary_ratio * (gyro_degrees[2] - accelerometer_z_rotation));

    // I dont want to track how many times the degrees went over the 360 degree mark, no point.
    while (gyro_degrees[0] > 180.0) {
        gyro_degrees[0] -= 360.0;
    }
    while (gyro_degrees[0] < -180.0) {
        gyro_degrees[0] += 360.0;
    }

    while (gyro_degrees[1] > 180.0) {
        gyro_degrees[1] -= 360.0;
    }
    while (gyro_degrees[1] < -180.0) {
        gyro_degrees[1] += 360.0;
    }

    while (gyro_degrees[2] > 180.0) {
        gyro_degrees[2] -= 360.0;
    }
    while (gyro_degrees[2] < -180.0) {
        gyro_degrees[2] += 360.0;
    }
}

void output_magnetometer_measurement(){
    // For calibrating magnetometer
    printf(
        "%f,%f,%f\n",
        magnetometer_data[0],
        magnetometer_data[1],
        magnetometer_data[2]
    );
}

void control_esc(uint8_t x, uint8_t y){
    if(x<=50){
        x = 0;
    }else{
        x -= 50;
        // x *= 1;
    }
    // printf("value: %d\n",x);
    uint16_t value = setServoActivationPercent(x, 50, 150);
    TIM1->CCR1 = value;
    TIM1->CCR4 = value;
    TIM2->CCR1 = value;
    TIM2->CCR2 = value;
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

void extract_request_values(char *request, uint8_t request_size, uint8_t *x, uint8_t *y)
{
    uint8_t x_index = 1;
    uint8_t x_end_index = 0;

    for (uint8_t i = 1; i < request_size; i++)
    {
        if (request[i] == '/')
        {
            x_end_index = i;
            break;
        }
    }

    uint8_t y_index = x_end_index + 1;
    uint8_t y_end_index = request_size - 1;

    uint8_t x_length = x_end_index - x_index;
    char x_substring[x_length + 1];
    strncpy(x_substring, &request[x_index], x_length);
    x_substring[x_length] = '\0';
    *x = atoi(x_substring);

    uint8_t y_length = y_end_index - y_index + 1;
    char y_substring[y_length + 1];
    strncpy(y_substring, &request[y_index], y_length);
    y_substring[y_length] = '\0';
    *y = atoi(y_substring);
}

void fix_mag_axis(float *magnetometer_data)
{
    float temp = 0.0;
    temp = magnetometer_data[0];
    // y is x
    magnetometer_data[0] = magnetometer_data[1];
    // x is -y
    magnetometer_data[1] = -temp;
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

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
    hi2c1.Init.ClockSpeed = 100000;
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
    htim1.Init.Prescaler = 999;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 999;
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
    htim2.Init.Prescaler = 999;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999;
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
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

    /*Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PB0 PB1 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
    printf("-------------------Crashed\n");
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
