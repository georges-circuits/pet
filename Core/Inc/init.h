
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l0xx_hal.h"

extern TIM_HandleTypeDef htim2;


void SystemClock_Config(void);
void MX_TIM2_Init(void);
void MX_GPIO_Init(void);


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


void Error_Handler(void);


#define USR_BTN_Pin GPIO_PIN_13
#define USR_BTN_GPIO_Port GPIOC
#define USR_BTN_EXTI_IRQn EXTI4_15_IRQn


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
