/*
 * general_utils.h
 *
 *  Created on: Dec 23, 2020
 *      Author: george
 */

#ifndef INC_GENERAL_UTILS_H_
#define INC_GENERAL_UTILS_H_

#include "init.h"

uint8_t General_Check_IfTimedOut(uint32_t tick_timeout_start, uint32_t timeout);

uint16_t General_Convert_BytesInto_Uint16(uint8_t* bytes);
uint16_t General_Convert_BytesInto_Uint16_Reversed(uint8_t* bytes);
uint32_t General_Convert_BytesInto_Uint32(uint8_t* bytes);


typedef struct
{
	uint32_t value;
	uint32_t counter;
} LoopDivider_t;

void General_LoopDivider_Init(LoopDivider_t* handle, uint32_t value);
int General_LoopDivider(LoopDivider_t* handle);

#endif /* INC_GENERAL_UTILS_H_ */
