#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_exti.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_pwr.h"

#define LED_PIN                                GPIO_PIN_10
#define LED_GPIO_PORT                          GPIOB

// uint16_t
// GPIO_TypeDef *

// void set_pin_status(){
//     if()
//     __HAL_RCC_GPIOC_CLK_ENABLE();

// }

int main(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

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

    while (1)
    {
    }
}

// void SysTick_Handler(void)
// {
//   HAL_IncTick();
// }

// void NMI_Handler(void)
// {
// }

// void HardFault_Handler(void)
// {
//   while (1) {}
// }


// void MemManage_Handler(void)
// {
//   while (1) {}
// }

// void BusFault_Handler(void)
// {
//   while (1) {}
// }

// void UsageFault_Handler(void)
// {
//   while (1) {}
// }

// void SVC_Handler(void)
// {
// }


// void DebugMon_Handler(void)
// {
// }

// void PendSV_Handler(void)
// {
// }