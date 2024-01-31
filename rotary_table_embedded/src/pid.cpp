/*
 * pid.c
 *
 *  Created on: Jan 9, 2024
 *      Author: johnt
 */

#include "pid.h"

void PIDController_Init(PIDController* pid)
{
	/* Reset the state of the controller */
	pid->integrator = 0.0f;
	pid->differentiator = 0.0f;

	pid->prev_measurement = 0.0f;
	pid->prev_error = 0.0f;

	pid->out = 0.0f;
}

float PIDController_Update(PIDController* pid, float setpoint, float measurement)
{
	/* Calculate the error */
	float error = setpoint - measurement;

	/* Proportional term */
	float proportional = pid->Kp * error;

	/* Integral term */
	pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prev_error);

	/* Anti-windup via dynamic integrator clamping*/
	float lim_min_int, lim_max_int;

	if (pid->lim_max > proportional)
	{
		lim_max_int = pid->lim_max - proportional;
	}
	else
	{
		lim_max_int = 0.0f;
	}

	if (pid->lim_min < proportional)
	{
		lim_min_int = pid->lim_min - proportional;
	}
	else
	{
		lim_min_int = 0.0f;
	}

	/* Clamp the integrator */
	if (pid->integrator > lim_max_int)
	{
		pid->integrator = lim_max_int;
	}
	else if (pid->integrator < lim_min_int)
	{
		pid->integrator = lim_min_int;
	}

	/* Derivative term with low pass filter */

	/* Derivative-on-measurement */
	pid->differentiator = (2.0f * pid->Kd * (pid->prev_measurement - measurement)
						+ (2.0f * pid->tau - pid->T) * pid->differentiator)
					    / (2.0f * pid->tau + pid->T);

	/* Derivative-on-error */
	/*
	pid->differentiator = (2.0f * pid->Kd * (error - pid->prev_error)
						+ (2.0f * pid->tau - pid->T) * pid->differentiator)
						/ (2.0f * pid->tau + pid->T);
	*/

	/* Feed forward term */
	float feedforward = pid->Kff * setpoint;

	/* Compute the controller output and apply limits */
	pid->out = proportional + pid->integrator + pid->differentiator + feedforward;

	if (pid->out > pid->lim_max)
	{
		pid->out = pid->lim_max;
	}
	else if (pid->out < pid->lim_min)
	{
		pid->out = pid->lim_min;
	}

	/* Store the controller state */
	pid->prev_measurement = measurement;
	pid->prev_error = error;

	/* Return the controller output*/
	return pid->out;
}
