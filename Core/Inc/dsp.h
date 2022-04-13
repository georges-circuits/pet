/*
 * dsp.h
 *
 *  Created on: Feb 16, 2021
 *  Author: Jiri Manak, manakjiri.eu
 * 
 *  Copyright (c) 2021. All rights reserved.
 */

#ifndef INC_DSP_H_
#define INC_DSP_H_

#ifdef __cplusplus
extern "C" {
#endif


typedef float DSP_fp_t;
typedef unsigned int uint;

typedef struct
{
	DSP_fp_t* data;
	uint len;
} DSP_Data_t;

void DSP_Data_Init(DSP_Data_t *handle, uint samples);
void DSP_Data_Deinit(DSP_Data_t *handle);
uint DSP_Data_Index(DSP_Data_t *handle, int index);
#define DSP_Data_ForEach(handle, value) for(uint i = 0; (value) = (&((handle)->data[i])), i < (handle)->len; i++)

typedef struct
{
	DSP_Data_t data;
	uint head;
} DSP_CircData_t;

void DSP_CircData_Init(DSP_CircData_t *handle, uint samples);
void DSP_CircData_Deinit(DSP_CircData_t *handle);
void DSP_CircData_Put(DSP_CircData_t *handle, DSP_fp_t sample);
uint DSP_CircData_IndexRelative(DSP_CircData_t *handle, int index);
#define DSP_CircData_ValueHead(handle) (&(handle)->data.data[(handle)->head])
#define DSP_CircData_IndexRelative(handle, index) (DSP_Data_Index((&(handle)->data), index + (int)(handle)->head))
#define DSP_CircData_ValueRelative(handle, index) (&(handle)->data.data[DSP_CircData_IndexRelative(handle, index)])
/*
#define DSP_CircData_IterForw(handle, val) for(int i = (handle)->head, t = 0; t < (handle)->data.len && \
	((val) = DSP_CircData_ValueRelative(handle, i++)); t++)
#define DSP_CircData_IterBack(handle, val) for(int i = (handle)->head + 1, t = 0; t < (handle)->data.len && \
	((val) = DSP_CircData_ValueRelative(handle, i--)); t++)
*/


typedef struct
{
	DSP_CircData_t buff;
	DSP_fp_t value;
} DSP_MovingAvg_t;

void DSP_MovingAvg_Init(DSP_MovingAvg_t *handle, uint samples, DSP_fp_t init);
void DSP_MovingAvg_Deinit(DSP_MovingAvg_t *handle);
void DSP_MovingAvg_Put(DSP_MovingAvg_t *handle, DSP_fp_t value);


typedef struct
{
	DSP_CircData_t buff;
	DSP_fp_t alpha;
} DSP_MovingEWAvg_t;

void DSP_MovingEWAvg_Init(DSP_MovingEWAvg_t *handle, uint samples, DSP_fp_t alpha, DSP_fp_t init);
void DSP_MovingEWAvg_Deinit(DSP_MovingEWAvg_t *handle);
void DSP_MovingEWAvg_Put(DSP_MovingEWAvg_t *handle, DSP_fp_t value);
DSP_fp_t DSP_MovingEWAvg_Get_Alpha(DSP_MovingEWAvg_t *handle, DSP_fp_t alpha);
#define DSP_MovingEWAvg_Get(handle) DSP_MovingEWAvg_Get_Alpha(handle, (handle)->alpha)



// in order to be fast it does not actually compute the variance from the definition
// use the DSP_MovingVar_Get function to read the value
// the avg is a normal DSP_MovingAvg_t which can be used to obtain moving average
// no need to have a separate instance of DSP_MovingAvg_t
typedef struct
{
	DSP_MovingAvg_t avg;
	DSP_CircData_t buff;
	DSP_fp_t _cumulative;
} DSP_MovingVar_t;

void DSP_MovingVar_Init(DSP_MovingVar_t* handle, uint samples, DSP_fp_t init);
void DSP_MovingVar_Deinit(DSP_MovingVar_t* handle);
void DSP_MovingVar_Put(DSP_MovingVar_t *handle, DSP_fp_t value);
DSP_fp_t DSP_MovingVar_Get(DSP_MovingVar_t *handle);


typedef struct
{
	DSP_fp_t last;
	DSP_fp_t filter_coefficient;
	uint8_t first;
} DSP_SimpleLowPass_t;

void DSP_SimpleLowPass_Init(DSP_SimpleLowPass_t* handle, DSP_fp_t filter_coefficient);
DSP_fp_t DSP_SimpleLowPass(DSP_SimpleLowPass_t* handle, DSP_fp_t input);


typedef enum {
	BIQUAD_LOWPASS,
	BIQUAD_BANDPASS,
	BIQUAD_NOTCH,
} DSP_Biquad_Type_t;

typedef struct
{
	DSP_fp_t b0, b1, b2, a1, a2;
	DSP_fp_t x1, x2, y1, y2;
} DSP_Biquad_t;

void DSP_Biquad_Init(DSP_Biquad_t* filter, DSP_Biquad_Type_t filterType, DSP_fp_t center_freq, uint16_t refresh_rate, DSP_fp_t Q);
DSP_fp_t DSP_BiquadDF1(DSP_Biquad_t *filter, DSP_fp_t input);
DSP_fp_t DSP_BiquadDF2(DSP_Biquad_t *filter, DSP_fp_t input);


typedef struct
{
	DSP_fp_t min, max;
	DSP_fp_t decay_rate;
} DSP_AmplitudeDecaying_t;

void DSP_AmplitudeDecaying_Init(DSP_AmplitudeDecaying_t* handle, DSP_fp_t sample_frequency, DSP_fp_t minimum_frequency);
DSP_fp_t DSP_AmplitudeDecaying(DSP_AmplitudeDecaying_t* handle, DSP_fp_t new_value);


typedef struct
{
	DSP_fp_t* samples;
	uint16_t num_samples;
	uint8_t holdoff;
} DSP_Slope_t;

void DSP_Slope_Init(DSP_Slope_t* handle, uint16_t num_samples);
void DSP_Slope_DeInit(DSP_Slope_t* handle);
DSP_fp_t DSP_Slope(DSP_Slope_t* handle, DSP_fp_t new_value);


#ifdef __cplusplus
}
#endif

#endif /* INC_DSP_H_ */
