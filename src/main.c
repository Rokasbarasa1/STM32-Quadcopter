#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_exti.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_uart.h"

#include "../lib/printf/retarget.h"
#include "stdio.h"
#include "string.h"
#include "../lib/mpu6050/mpu6050.h"
#include "../lib/gy271_qmc5883l/gy271.h"
#include "../lib/bmp280/bmp280.h"

float hard_iron_correction[3] = {
    185.447609, 360.541288, 491.294615
};
float soft_iron_correction[3][3] = {
    {1.001470, 0.025460, -0.035586},
    {0.025460, 0.405497, -0.054355},
    {-0.035586, -0.054355, 1.219251}
};

float accelerometer_correction[3] = {
    0.068885,0.054472,0.952431
};
float gyro_correction[3] = {
    0.208137,-4.056841,0.413817
};

void SystemClock_Config(void);
void init_uart_1();
void init_leds();
void init_i2c_1();
I2C_HandleTypeDef i2c_1;
UART_HandleTypeDef uart_1;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    init_uart_1();
    RetargetInit(&uart_1);
    init_leds();
    init_i2c_1();

    printf("\n\nInitializing...\n");

    init_mpu6050(&i2c_1, 1, accelerometer_correction, gyro_correction);
    init_gy271(&i2c_1, 1, hard_iron_correction, soft_iron_correction);
    init_bmp280(&i2c_1);

    float magnetometer_data[] = {0,0,0};
    gy271_magnetometer_readings_micro_teslas(magnetometer_data);

    printf("mag: %f\n", magnetometer_data[0]);
    float preassure_data[] = {0,0,0};
    bmp280_preassure_float(preassure_data);
    bmp280_temperature_float(preassure_data);


    float data[] = {0,0,0};
    while (1){
        // printf("PRE %d %f\n", 1, 1.23);

        // printf("DATA X:%6.2f Y:%6.2f Z:%6.2f\n", data[0],data[1],data[2]);
        // mpu6050_accelerometer_readings_float(data);
        // printf("DATA X:%6.2f Y:%6.2f Z:%6.2f\n", data[0],data[1],data[2]);
        // printf("FUCKED UP I2C\r\n");

        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
        HAL_Delay(250);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
        HAL_Delay(250);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Initializes the RCC Oscillators according to the specified parameters
    //in the RCC_OscInitTypeDef structure.
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        // Error_Handler();
    }

    // Initializes the CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        // Error_Handler();
    }
}

/**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
*/
void init_uart_1(){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    uart_1.Instance = USART1;
    uart_1.Init.BaudRate = 115200;
    uart_1.Init.WordLength = UART_WORDLENGTH_8B;
    uart_1.Init.StopBits = UART_STOPBITS_1;
    uart_1.Init.Parity = UART_PARITY_NONE;
    uart_1.Init.Mode = UART_MODE_TX_RX;
    uart_1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&uart_1) != HAL_OK){

    }
}

/**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
*/
void init_i2c_1(){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    i2c_1.Instance = I2C1;
    i2c_1.Init.ClockSpeed = 400000;
    i2c_1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    i2c_1.Init.OwnAddress1 = 0;
    i2c_1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c_1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c_1.Init.OwnAddress2 = 0;
    i2c_1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c_1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&i2c_1) != HAL_OK){
        printf("FUCKED UP I2C");
        // Error_Handler();
    }

    HAL_I2CEx_ConfigAnalogFilter(&i2c_1, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&i2c_1, 0);

}

void init_leds(){
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_B = {0};
    GPIO_B.Pin = GPIO_PIN_13;
    GPIO_B.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_B.Pull = GPIO_NOPULL;
    GPIO_B.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_B);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    GPIO_InitTypeDef GPIO_A = {0};
    GPIO_A.Pin = GPIO_PIN_11;
    GPIO_A.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_A.Pull = GPIO_NOPULL;
    GPIO_A.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_A);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
}


void SysTick_Handler(void)
{
  HAL_IncTick();
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1) {}
}


void MemManage_Handler(void)
{
  while (1) {}
}

void BusFault_Handler(void)
{
  while (1) {}
}

void UsageFault_Handler(void)
{
  while (1) {}
}

void SVC_Handler(void)
{
}


void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}