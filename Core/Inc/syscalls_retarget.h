/*
 * syscalls_retarget.h
 *
 *  Created on: Dec 23, 2020
 *      Author: george
 */

#ifndef INC_SYSCALLS_RETARGET_H_
#define INC_SYSCALLS_RETARGET_H_

// All credit to Carmine Noviello for this code
// https://github.com/cnoviello/mastering-stm32/blob/master/nucleo-f030R8/system/include/retarget/retarget.h

#ifndef _RETARGET_H__
#define _RETARGET_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l0xx_hal.h"
#include <sys/stat.h>

void RetargetInit(UART_HandleTypeDef *huart);
int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char* ptr, int len);
int _fstat(int fd, struct stat* st);

#ifdef __cplusplus
}
#endif

#endif //#ifndef _RETARGET_H__

#endif /* INC_SYSCALLS_RETARGET_H_ */
