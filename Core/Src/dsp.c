/*
 * dsp.c
 *
 *  Created on: Feb 15, 2021
 *  Author: Jiri Manak, manakjiri.eu
 * 
 *  Copyright (c) 2021. All rights reserved.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "dsp.h"


#define PI_FLOAT  	3.14159265358979323846f
#define BIQUAD_Q 	1.0f / sqrtf(2.0f)     /* quality factor - 2nd order butterworth */


void DSP_Data_Init(DSP_Data_t *handle, uint samples)
{
	handle->data = calloc(sizeof(DSP_fp_t), samples);
	handle->len = samples;
}

void DSP_Data_Deinit(DSP_Data_t *handle)
{
	if (handle->data)
	{
		free(handle->data);
		handle->data = NULL;
	}
	handle->len = 0;
}

void DSP_Data_SetAll(DSP_Data_t *handle, DSP_fp_t value)
{
	DSP_fp_t *val;
	DSP_Data_ForEach(handle, val)
		*val = value;
}

// returns a valid index within handle->data
uint DSP_Data_Index(DSP_Data_t *handle, int index)
{
	if (index < 0)
		return index + handle->len;
	while (index >= handle->len)
		index -= handle->len;
	//printf("index %i\n", index);
	return index;
}



void DSP_CircData_Init(DSP_CircData_t *handle, uint samples)
{
	DSP_Data_Init(&handle->data, samples);
	handle->head = 0;
}

void DSP_CircData_Deinit(DSP_CircData_t *handle)
{
	DSP_Data_Deinit(&handle->data);
}

void DSP_CircData_Put(DSP_CircData_t *handle, DSP_fp_t sample)
{
	handle->data.data[handle->head] = sample;
	handle->head++;
	if (handle->head >= handle->data.len)
		handle->head = 0;
}

// returns a valid index within handle->data.data relative to the head of the buffer
/*uint DSP_CircData_IndexRelative(DSP_CircData_t *handle, int index)
{
	return DSP_Data_Index(&handle->data, index + (int)handle->head);
}*/


void DSP_MovingAvg_Init(DSP_MovingAvg_t *handle, uint samples, DSP_fp_t init)
{
	DSP_CircData_Init(&handle->buff, samples);
	DSP_Data_SetAll(&handle->buff.data, init);
	handle->value = init;
}

void DSP_MovingAvg_Deinit(DSP_MovingAvg_t *handle)
{
	DSP_CircData_Deinit(&handle->buff);
}

void DSP_MovingAvg_Put(DSP_MovingAvg_t *handle, DSP_fp_t value)
{
	value /= handle->buff.data.len;

	// add new value
	handle->value += value;
	// subtract the oldest value (which will be where the head is now)
	handle->value -= *DSP_CircData_ValueHead(&handle->buff);
	// put in the new value (this puts it at the head position and moves the head forward)
	DSP_CircData_Put(&handle->buff, value);
}


// 0 < alpha < 1; samples >= 2
void DSP_MovingEWAvg_Init(DSP_MovingEWAvg_t *handle, uint samples, DSP_fp_t alpha, DSP_fp_t init)
{
	handle->alpha = alpha;
	DSP_CircData_Init(&handle->buff, samples);
	DSP_Data_SetAll(&handle->buff.data, init);
}

void DSP_MovingEWAvg_Deinit(DSP_MovingEWAvg_t *handle)
{
	DSP_CircData_Deinit(&handle->buff);
}

void DSP_MovingEWAvg_Put(DSP_MovingEWAvg_t *handle, DSP_fp_t value)
{
	DSP_CircData_Put(&handle->buff, value);
}

DSP_fp_t DSP_MovingEWAvg_Get_Alpha(DSP_MovingEWAvg_t *handle, DSP_fp_t alpha)
{
	DSP_fp_t ret;
	// according to https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
	// but without recursion

	// get the oldest sample (the one where head is now - head counter gets incremented after value insertion)
	ret = *DSP_CircData_ValueHead(&handle->buff);

	// get all the remaining samples starting with the second oldest and compute the result
	for (int i = 1; i < handle->buff.data.len; i++)
		ret = (alpha * (*DSP_CircData_ValueRelative(&handle->buff, i))) + ((1 - alpha) * ret);

	return ret;
}



void DSP_MovingVar_Init(DSP_MovingVar_t* handle, uint samples, DSP_fp_t init)
{
	DSP_MovingAvg_Init(&handle->avg, samples, init);
	DSP_CircData_Init(&handle->buff, samples);
	handle->_cumulative = 0;
}

void DSP_MovingVar_Deinit(DSP_MovingVar_t* handle)
{
	DSP_MovingAvg_Deinit(&handle->avg);
	DSP_CircData_Deinit(&handle->buff);
}

void DSP_MovingVar_Put(DSP_MovingVar_t *handle, DSP_fp_t value)
{
	// update the moving average
	DSP_MovingAvg_Put(&handle->avg, value);

	// compute "variance" of this iteration
	DSP_fp_t var = (value - handle->avg.value); var *= var;

	// add new value
	handle->_cumulative += var;
	// subtract the oldest value (which will be where the head is now)
	handle->_cumulative -= *DSP_CircData_ValueHead(&handle->buff);
	// put in the new value (this puts it at the head position and moves the head forward)
	DSP_CircData_Put(&handle->buff, var);
}

DSP_fp_t DSP_MovingVar_Get(DSP_MovingVar_t *handle)
{
	return sqrtf(handle->_cumulative / (handle->buff.data.len - 1));
}




void DSP_SimpleLowPass_Init(DSP_SimpleLowPass_t* handle, DSP_fp_t filter_coefficient)
{
	if (filter_coefficient > 1.0f)
		filter_coefficient = 1.0f;
	if (filter_coefficient < 0.0f)
		filter_coefficient = 0.0f;

	handle->filter_coefficient = filter_coefficient;
	handle->last = 0.0;
	handle->first = 0;
}

DSP_fp_t DSP_SimpleLowPass(DSP_SimpleLowPass_t* handle, DSP_fp_t input)
{
	if (!handle->first)
	{
		handle->first = 1;
		handle->last = input;
	}
	DSP_fp_t ret = input * handle->filter_coefficient + handle->last * (1.0f - handle->filter_coefficient);
	handle->last = ret;
	return ret;
}


void DSP_Biquad_Init(DSP_Biquad_t* filter, DSP_Biquad_Type_t filterType, DSP_fp_t center_freq, uint16_t refresh_rate, DSP_fp_t Q)
{
    // setup variables
    const DSP_fp_t omega = 2.0f * PI_FLOAT * (center_freq / refresh_rate);
    const DSP_fp_t sn = sinf(omega);
    const DSP_fp_t cs = cosf(omega);
    const DSP_fp_t alpha = sn / (2.0f * Q);

    DSP_fp_t b0 = 0, b1 = 0, b2 = 0, a0 = 0, a1 = 0, a2 = 0;

    switch (filterType) {
    case BIQUAD_LOWPASS:
        // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
        // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
        b0 = (1 - cs) * 0.5f;
        b1 = 1 - cs;
        b2 = (1 - cs) * 0.5f;
        a0 = 1 + alpha;
        a1 = -2 * cs;
        a2 = 1 - alpha;
        break;
    case BIQUAD_NOTCH:
        b0 =  1;
        b1 = -2 * cs;
        b2 =  1;
        a0 =  1 + alpha;
        a1 = -2 * cs;
        a2 =  1 - alpha;
        break;
    case BIQUAD_BANDPASS:
        b0 = alpha;
        b1 = 0;
        b2 = -alpha;
        a0 = 1 + alpha;
        a1 = -2 * cs;
        a2 = 1 - alpha;
        break;
    }

    // precompute the coefficients
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    // zero initial samples
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

/* Computes a DSP_Biquad_t filter on a sample (slightly less precise than df2 but works in dynamic mode) */
DSP_fp_t DSP_BiquadDF1(DSP_Biquad_t *filter, DSP_fp_t input)
{
    /* compute result */
    const DSP_fp_t result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    /* shift x1 to x2, input to x1 */
    filter->x2 = filter->x1;
    filter->x1 = input;

    /* shift y1 to y2, result to y1 */
    filter->y2 = filter->y1;
    filter->y1 = result;

    return result;
}

/* Computes a DSP_Biquad_t filter in direct form 2 on a sample (higher precision but can't handle changes in coefficients */
DSP_fp_t DSP_BiquadDF2(DSP_Biquad_t *filter, DSP_fp_t input)
{
    const DSP_fp_t result = filter->b0 * input + filter->x1;
    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;
    return result;
}


// this is an attempt to measure amplitude based on min-max
// and constant decay rate which slowly nullifies the measured min and max amplitude
// initialize the DSP_AmplitudeDecaying_t which measures the pk-pk amplitude of a signal
// without storing an array of previous values.
// minimum_frequency is the lowest expected frequency of the measured signal
// if the signal frequency goes below the minimum_frequency the amplitude measurement won't be accurate
// sample_frequency must be multiple times higher than the minimum_frequency
void DSP_AmplitudeDecaying_Init(DSP_AmplitudeDecaying_t* handle, DSP_fp_t sample_frequency, DSP_fp_t minimum_frequency)
{
	handle->max = 0.0f;
	handle->min = 0.0f;
	handle->decay_rate = 1 / (sample_frequency / minimum_frequency);

	if (handle->decay_rate > 0.95f)
		handle->decay_rate = 0.95f;
}

DSP_fp_t DSP_AmplitudeDecaying(DSP_AmplitudeDecaying_t* handle, DSP_fp_t new_value)
{
	if (new_value > handle->max)
		handle->max = new_value;
	else
	{
		DSP_fp_t decay = (handle->max - new_value) * handle->decay_rate;
		if (decay > 0.0f)
			handle->max -= decay;
	}

	if (new_value < handle->min)
		handle->min = new_value;
	else
	{
		DSP_fp_t decay = (new_value - handle->min) * handle->decay_rate;
		if (decay > 0.0f)
			handle->min += decay;
	}
	return handle->max - handle->min;
}


void DSP_Slope_Init(DSP_Slope_t* handle, uint16_t num_samples)
{
	handle->num_samples = num_samples;
	handle->holdoff = num_samples;
	handle->samples = calloc(num_samples, sizeof(DSP_fp_t));
}

void DSP_Slope_DeInit(DSP_Slope_t* handle)
{
	if (handle->samples)
		free(handle->samples);

	handle->samples = NULL;
}

// returns climb rate in the units of new_value
DSP_fp_t DSP_Slope(DSP_Slope_t* handle, DSP_fp_t new_value)
{
	// calculate the average slope
	DSP_fp_t slope = new_value - handle->samples[0];
	for (int i = 0; i < handle->num_samples - 1; i++)
		slope += handle->samples[i] - handle->samples[i + 1];
	slope /= (DSP_fp_t)handle->num_samples;

	// update sample history
	for (int i = handle->num_samples - 2; i >= 0; i--)
		handle->samples[i + 1] = handle->samples[i];
	handle->samples[0] = new_value;

	if (handle->holdoff > 0)
	{
		handle->holdoff--;
		return 0.0f;
	}
	else
		return slope;
}









