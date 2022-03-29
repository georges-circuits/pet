/*
 * general_utils.c
 *
 *  Created on: Dec 23, 2020
 *      Author: george
 */

#include "general_utils.h"

// accounts for Tick overflow
uint8_t General_Check_IfTimedOut(uint32_t tick_timeout_start, uint32_t timeout)
{
	uint32_t tick = HAL_GetTick();
	if (tick - tick_timeout_start >= timeout || ((int64_t)tick - (int64_t)tick_timeout_start < 0 &&
			((int64_t)tick + (int64_t)UINT32_MAX) - (int64_t)tick_timeout_start >= (int64_t)timeout))
		return 1;
	else
		return 0;
}

uint16_t General_Convert_BytesInto_Uint16(uint8_t* bytes)
{
	uint16_t ret = 0;
	ret |= ((uint16_t)bytes[0]) << 8;
	ret |= ((uint16_t)bytes[1]) << 0;
	return ret;
}

uint16_t General_Convert_BytesInto_Uint16_Reversed(uint8_t* bytes)
{
	uint16_t ret = 0;
	ret |= ((uint16_t)bytes[1]) << 8;
	ret |= ((uint16_t)bytes[0]) << 0;
	return ret;
}

uint32_t General_Convert_BytesInto_Uint32(uint8_t* bytes)
{
	uint32_t ret = 0;
	ret |= ((uint32_t)bytes[0]) << 24;
	ret |= ((uint32_t)bytes[1]) << 16;
	ret |= ((uint32_t)bytes[2]) << 8;
	ret |= ((uint32_t)bytes[3]) << 0;
	return ret;
}


void General_LoopDivider_Init(LoopDivider_t* handle, uint32_t value)
{
	handle->counter = 0;
	handle->value = value;
}

int General_LoopDivider(LoopDivider_t* handle)
{
	if (handle->counter++ > handle->value)
	{
		handle->counter = 0;
		return 1;
	}
	return 0;
}





