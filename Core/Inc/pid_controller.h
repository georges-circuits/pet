/*
 * pid_controller.h
 *
 *  Created on: Dec 23, 2020
 *      Author: george
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "init.h"

typedef float PID_fp_t;

typedef struct
{
    // tuning parameters
    PID_fp_t P_gain;
    PID_fp_t I_gain;
    PID_fp_t D_gain;

    // other configuration
    PID_fp_t val_max;
    PID_fp_t val_min;
    PID_fp_t deadband;
    PID_fp_t ramp_k;
    PID_fp_t I_deadband;
    PID_fp_t I_divdecr;
    PID_fp_t I_absmax;

    // persistent values
    PID_fp_t err_cumulative;
    PID_fp_t err_last;
    PID_fp_t time_last;
    PID_fp_t last_setpoint;

    // loop values
    PID_fp_t p, i, d;
} PID_t;

void PID_Init(PID_t* pid);
void PID_Set_Gains(PID_t* pid, PID_fp_t proportional, PID_fp_t integral, PID_fp_t derivative);
void PID_Set_MinMax(PID_t* pid, PID_fp_t val_min, PID_fp_t val_max);
void PID_Set_Deadband(PID_t* pid, PID_fp_t deadband);
void PID_Set_I_Deadband(PID_t* pid, PID_fp_t deadband);
void PID_Set_I_DivideDecrement(PID_t* pid, PID_fp_t decr);
void PID_Set_I_AbsMax(PID_t* pid, PID_fp_t max);
void PID_Set_SetpointRamp(PID_t* pid, PID_fp_t k);
void PID_Reset_CumulativeValues(PID_t* pid);
PID_fp_t PID_Get_LastError(PID_t* pid);
PID_fp_t PID_Update_Setpoint_TimeDelta(PID_t* pid, PID_fp_t setpoint, PID_fp_t current, PID_fp_t time_delta);
PID_fp_t PID_Update_Setpoint(PID_t* pid, PID_fp_t setpoint, PID_fp_t current);

#ifdef __cplusplus
}
#endif

#endif /* INC_PID_CONTROLLER_H_ */
