
#ifndef __INIT_H
#define __INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l0xx_hal.h"
#include "pin_defs.h"

extern TIM_HandleTypeDef htim2;


void SystemClock_Config(void);
void MX_TIM2_Init(void);
void MX_GPIO_Init(void);


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
