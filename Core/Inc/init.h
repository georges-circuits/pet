
#ifndef __INIT_H
#define __INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l0xx_hal.h"
#include "pin_defs.h"

extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;


void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM6_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART4_UART_Init(void);
void MX_USART5_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
