/*
 * stm_adc.h
 *
 *  Created on: Feb 7, 2021
 *      Author: george
 */

#ifndef INC_STM_ADC_H_
#define INC_STM_ADC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "init.h"

#define STM32ADC_FULLSCALE			4095.0f
#define STM32ADC_FULLSCALEVOLTAGE	3.3f

typedef enum
{
	STM32ADC_CHANNEL_IR,
	STM32ADC_CHANNEL_BATT,
	STM32ADC_CHANNEL_VREF,
	STM32ADC_CHANNELS
} STM32ADC_Channel_t;


typedef struct __attribute__((__packed__))
{
	float offset;
	float scale;
} STM32ADC_Calibration_t;

#define STM32ADC_CALIBRATION_SIZE (sizeof(STM32ADC_Calibration_t) * STM32ADC_CHANNELS)

typedef struct
{
	ADC_HandleTypeDef* hadc;
	uint16_t raw[STM32ADC_CHANNELS];
	STM32ADC_Calibration_t calibration[STM32ADC_CHANNELS];
	float reference;

	uint32_t samples_per_second;
	uint32_t samples_per_second_count;
	uint32_t tick;
	uint8_t timer_running;
} STM32ADC_t;

void STM32ADC_Init(ADC_HandleTypeDef* hadc);
void STM32ADC_Start();
void STM32ADC_Stop();
void STM32ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
float STM32ADC_GetReading(STM32ADC_Channel_t channel);
float STM32ADC_GetReadingUpdated(STM32ADC_Channel_t channel);
uint32_t STM32ADC_GetSPS();
void STM32ADC_Calibrate_Scale(STM32ADC_Channel_t channel, uint16_t samples_to_avg, float expected_value);
void STM32ADC_Calibrate_Offset(STM32ADC_Channel_t channel, uint16_t samples_to_avg, float expected_value);

#ifdef __cplusplus
}
#endif

#endif /* INC_STM_ADC_H_ */
