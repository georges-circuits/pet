/*
 * stm_adc.c
 *
 *  Created on: Feb 7, 2021
 *      Author: george
 */

#include "stm_adc.h"
#include "general_utils.h"
#include <string.h>

#define STM32ADC_REFERENCE		1.25f

static const float default_offset = 0.0f;
static const float default_scale = 1.0f;

static volatile STM32ADC_t adc_handle;

void _STM32ADC_Init(volatile STM32ADC_t* adc, ADC_HandleTypeDef* hadc)
{
	STM32ADC_Calibration_t default_calibration;
	default_calibration.offset = default_offset;
	default_calibration.scale = default_scale;

	for (int i = 0; i < STM32ADC_CHANNELS; i++)
	{
		adc->raw[i] = 0;
		adc->calibration[i] = default_calibration;
	}
	adc->reference = STM32ADC_FULLSCALEVOLTAGE;
	adc->hadc = hadc;
	adc->samples_per_second_count = 0;
	adc->samples_per_second = 0;
	adc->tick = HAL_GetTick();
	adc->timer_running = 0;
}

void _STM32ADC_Start(volatile STM32ADC_t* adc)
{
	adc->timer_running = 1;
	HAL_ADC_Start_DMA(adc->hadc, (uint32_t*)adc->raw, STM32ADC_CHANNELS);
}

void _STM32ADC_Stop(volatile STM32ADC_t* adc)
{
	HAL_ADC_Stop_DMA(adc->hadc);
	adc->timer_running = 0;
	adc->samples_per_second = 0;
}

void _STM32ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc, volatile STM32ADC_t* adc)
{
	if (adc->hadc->Instance == hadc->Instance)
	{
		adc->samples_per_second_count++;
		if (General_Check_IfTimedOut(adc->tick, 1000))
		{
			adc->tick = HAL_GetTick();
			adc->samples_per_second = adc->samples_per_second_count;
			adc->samples_per_second_count = 0;
		}
	}
}

float _STM32ADC_GetReading(volatile STM32ADC_t* adc, STM32ADC_Channel_t channel)
{
	return (((float)adc->raw[channel] + adc->calibration[channel].offset) * adc->calibration[channel].scale);
}

float _STM32ADC_GetVrefCorrection(volatile STM32ADC_t* adc)
{
	return (STM32ADC_REFERENCE * STM32ADC_FULLSCALE) / (_STM32ADC_GetReading(adc, STM32ADC_CHANNEL_VREF) * adc->reference);
}

float _STM32ADC_GetReadingReferenceCorrected(volatile STM32ADC_t* adc, STM32ADC_Channel_t channel)
{
	return _STM32ADC_GetReading(adc, channel) * _STM32ADC_GetVrefCorrection(adc) * (adc->reference / STM32ADC_FULLSCALE);
}

float _STM32ADC_GetReadingReferenceCorrectedUpdated(volatile STM32ADC_t* adc, STM32ADC_Channel_t channel)
{
	uint32_t sps_last = adc->samples_per_second_count;
	while (sps_last == adc->samples_per_second_count) {}
	return _STM32ADC_GetReadingReferenceCorrected(adc, channel);
}

float _STM32ADC_GetReadingAverage(volatile STM32ADC_t* adc, STM32ADC_Channel_t channel, uint16_t samples_to_avg)
{
	uint32_t sps_last = adc->samples_per_second_count;
	uint16_t samples = 0;
	float avg = 0;
	while(samples < samples_to_avg)
	{
		if (sps_last != adc->samples_per_second_count)
		{
			sps_last = adc->samples_per_second_count;
			samples++;
			avg += _STM32ADC_GetReadingReferenceCorrected(adc, channel);
		}
	}
	return avg / (float)samples_to_avg;
}

void _STM32ADC_Calibrate_Scale(volatile STM32ADC_t* adc, STM32ADC_Channel_t channel, uint16_t samples_to_avg, float expected_value)
{
	adc->calibration[channel].scale = default_scale;
	float avg = _STM32ADC_GetReadingAverage(adc, channel, samples_to_avg);
	float new_scale = expected_value / avg;
	adc->calibration[channel].scale = new_scale;
}

void _STM32ADC_Calibrate_Offset(volatile STM32ADC_t* adc, STM32ADC_Channel_t channel, uint16_t samples_to_avg, float expected_value)
{
	adc->calibration[channel].offset = default_offset;
	float avg = _STM32ADC_GetReadingAverage(adc, channel, samples_to_avg);
	float new_offset = expected_value / avg;
	adc->calibration[channel].offset = new_offset;
}







void STM32ADC_Init(ADC_HandleTypeDef* hadc)
{
	_STM32ADC_Init(&adc_handle, hadc);
}

void STM32ADC_Start()
{
	_STM32ADC_Start(&adc_handle);
}

void STM32ADC_Stop()
{
	_STM32ADC_Stop(&adc_handle);
}

// should be called by HAL_ADC_ConvCpltCallback
void STM32ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	_STM32ADC_ConvCpltCallback(hadc, &adc_handle);
}

// applies the calibration and corrects for variations in vref, uses factory trim if available
// returns the voltage in Volts with respect to VREF
float STM32ADC_GetReading(STM32ADC_Channel_t channel)
{
	return _STM32ADC_GetReadingReferenceCorrected(&adc_handle, channel);
}

// just like STM32ADC_GetReading but this one waits for new update
float STM32ADC_GetReadingUpdated(STM32ADC_Channel_t channel)
{
	return _STM32ADC_GetReadingReferenceCorrectedUpdated(&adc_handle, channel);
}

uint32_t STM32ADC_GetSPS()
{
	return adc_handle.samples_per_second;
}

// expected_value - one returned by STM32ADC_GetReading
// this will block for samples_to_avg times the sample rate, the adc must be sampling or this will hang!
void STM32ADC_Calibrate_Scale(STM32ADC_Channel_t channel, uint16_t samples_to_avg, float expected_value)
{
	_STM32ADC_Calibrate_Scale(&adc_handle, channel, samples_to_avg, expected_value);
}

// expected_value - one returned by STM32ADC_GetReading
// this will block for samples_to_avg times the sample rate, the adc must be sampling or this will hang!
void STM32ADC_Calibrate_Offset(STM32ADC_Channel_t channel, uint16_t samples_to_avg, float expected_value)
{
	_STM32ADC_Calibrate_Offset(&adc_handle, channel, samples_to_avg, expected_value);
}
