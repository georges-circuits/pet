/*
 * pid_controller.c
 *
 *  Created on: Dec 23, 2020
 *      Author: george
 */

#include "pid_controller.h"

static const PID_t struct_reset = {0};

static inline PID_fp_t PID_Abs(PID_fp_t val)
{
	return val > 0 ? val : -val;
}

static inline int PID_Sign(PID_fp_t val)
{
	if (val > 0) return 1;
	if (val < 0) return -1;
	return 0;
}

static inline int PID_ZeroCross(PID_fp_t val1, PID_fp_t val2)
{
	int s1 = PID_Sign(val1), s2 = PID_Sign(val2);
	if (s1 > s2) return -1;
	if (s1 < s2) return 1;
	return 0;

}

// reset the PID_t structure to a known state
// to make the PID controller do anything you need to use the
// PID_Set_Gains and PID_Set_MinMax
void PID_Init(PID_t* pid)
{
    *pid = struct_reset;
    pid->P_gain = 1.0;
}

void PID_Set_Gains(PID_t* pid, PID_fp_t proportional, PID_fp_t integral, PID_fp_t derivative)
{
    pid->P_gain = proportional;
    pid->I_gain = integral;
    pid->D_gain = derivative;
}

void PID_Set_MinMax(PID_t* pid, PID_fp_t val_min, PID_fp_t val_max)
{
	pid->val_min = val_min;
	pid->val_max = val_max;
}

void PID_Set_Deadband(PID_t* pid, PID_fp_t deadband)
{
	pid->deadband = deadband;
}

// if the error is below this value then the I term gets zeroed
// prevents overshoot after integral windup
void PID_Set_I_Deadband(PID_t* pid, PID_fp_t deadband)
{
	pid->I_deadband = deadband > 0.0 ? deadband : 0.0;
}

// decreases integral windup
void PID_Set_I_DivideDecrement(PID_t* pid, PID_fp_t decr)
{
	pid->I_divdecr = decr;
}

void PID_Set_I_AbsMax(PID_t* pid, PID_fp_t max)
{
	pid->I_absmax = max > 0.0 ? max : 0.0;
}

void PID_Set_SetpointRamp(PID_t* pid, PID_fp_t k)
{
	pid->ramp_k = k > 0 ? k : 0;
}

void PID_Reset_CumulativeValues(PID_t* pid)
{
	pid->err_cumulative = 0.0f;
	pid->err_last = 0.0f;
}

PID_fp_t PID_Get_LastError(PID_t* pid)
{
	return pid->err_last;
}


// setpoint - the desired value
// current - the actual measured value
// time_delta - time in seconds since the last update
// returns the computed control value
// mainly used for simulation
PID_fp_t PID_Update_Setpoint_TimeDelta(PID_t* pid, PID_fp_t setpoint, PID_fp_t current, PID_fp_t time_delta)
{
	PID_fp_t err, abs_err, out;

	// ramp the setpoint if the ramp is enabled
	if (pid->ramp_k > 0 && PID_Abs(setpoint - pid->last_setpoint) > pid->ramp_k)
		setpoint = pid->last_setpoint + (setpoint > pid->last_setpoint ? pid->ramp_k : -pid->ramp_k);

    pid->last_setpoint = setpoint;

    // calculate the error
    err = setpoint - current;
    abs_err = PID_Abs(err);

	// disable if within the deadband
    if (abs_err < pid->deadband)
    	return 0.0;

    /* when pid->I_absmax == 0.0 this works as normal cumulative error
     * however if the limiter is enabled then the only time the current error gets
     * added to the cumulative value is when that error is within the limit or,
     * if it is already over that limit, only added if that will cause
     * the cumulative to grow smaller
     */
    if (pid->I_absmax == 0.0 || PID_Abs(pid->i) < pid->I_absmax ||
    		PID_Abs(pid->err_cumulative + err) < PID_Abs(pid->err_cumulative))
    	pid->err_cumulative += err;


    // zero cumulative error if I_deadband is enabled and the error is within it
    if (pid->I_deadband > 0.0 && (abs_err <= pid->I_deadband || PID_ZeroCross(err, pid->err_last) != 0))
    	pid->err_cumulative = 0.0;

    // apply gradual decrease to I over time if enabled
    if (pid->I_divdecr > 1.0)
    	pid->err_cumulative /= pid->I_divdecr;

    // calculate PIDs
    pid->p = err * pid->P_gain;
    pid->i = pid->err_cumulative * time_delta * pid->I_gain;
    pid->d = (((err - pid->err_last) / time_delta) * pid->D_gain);

    pid->err_last = err;

    // calculate the output from PIDs
    out = pid->p + pid->i + pid->d;

    // clamp the output according to min and max
    if (out > pid->val_max)
    	out = pid->val_max;
    if (out < pid->val_min)
        out = pid->val_min;

    // zero output if within the deadband
    if (PID_Abs(out) < pid->deadband)
    	out = 0.0;

    return out;
}

PID_fp_t PID_Update_Setpoint(PID_t* pid, PID_fp_t setpoint, PID_fp_t current)
{
	// calculate dt - platform specific
	PID_fp_t time = ((PID_fp_t)HAL_GetTick()) / 1000.0;
	PID_fp_t dt = time - pid->time_last;
	pid->time_last = time;

	return PID_Update_Setpoint_TimeDelta(pid, setpoint, current, dt);
}



