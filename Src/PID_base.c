/*
 * pid.c
 *
 *  Created on: Dec 3, 2020
 *      Author: by
 */

#include <PID_base.h>
#include <math.h>

#define DYNAMIC_INTEGRATOR 0



void PID_Init(PID_Struct* pid)
{
	pid->P = 0;
	pid->I = 0;
	pid->D = 0;
	pid->F = 0;
	pid->maxIOutput = 0;
	pid->maxError = 0;
	pid->errorSum = 0;
	pid->maxOutput = 0;
	pid->minOutput = 0;
	pid->setpoint = 0;
	pid->lastActual = 0;
	pid->firstRun = 1;
	pid->reversed = 0;
	pid->outputRampRate = 0;
	pid->outputDescentRate = 0;
	pid->lastOutput = 0;
	pid->outputFilter = 0;
	pid->setpointRange = 0;
	pid->deadTime = 0;
	pid->frequency = 50.0;
}

void checkSigns(PID_Struct* pid)
{
	if(pid->reversed)
	{
		if(pid->P > 0)
			pid->P *= -1;
		if(pid->I > 0)
			pid->I *= -1;
		if(pid->D > 0)
			pid->D *= -1;
		if(pid->F > 0)
			pid->F *= -1;
	}

	else
	{
		if(pid->P < 0)
			pid->P *= -1;
		if(pid->I < 0)
			pid->I *= -1;
		if(pid->D < 0)
			pid->D *= -1;
		if(pid->F < 0)
			pid->F *= -1;
	}
}

float clamp(float value, float min, float max)
{
	if (value > max)
	{
		return max;
	}
	if (value < min)
	{
		return min;
	}
	return value;
}

uint8_t bounded(float value, float min, float max)
{
	return (min < value) && (value < max);
}

void PID_setPID(PID_Struct* pid, float p, float i, float d)
{
	pid->P = p;
	pid->I = i;
	pid->D = d;
	checkSigns(pid);
}

void PID_setPIDF(PID_Struct* pid, float p, float i, float d, float f)
{
	pid->P = p;
	pid->I = i;
	pid->D = d;
	pid->F = f;
	checkSigns(pid);
}

void PID_setF(PID_Struct* pid, float f)
{
	pid->F = f;
	checkSigns(pid);
}
//TODO: Implement dynamic integrator
void PID_setMaxIOutput(PID_Struct* pid, float maximum)
{
	pid->maxIOutput = maximum;
	if(pid->I != 0)
	{
		pid->maxError = pid->maxIOutput / pid->I;
	}
}

void PID_setMinIOutput(PID_Struct* pid, float minimum)
{
	pid->minIOutput = minimum;
	if(pid->I != 0)
	{
		pid->maxError = pid->maxIOutput / pid->I;
	}
}


void PID_setOutputLimits(PID_Struct* pid, float min, float max)
{
	if(max > min)
	{
		pid->maxOutput = max;
		pid->minOutput = min;
	}

	if(pid->maxIOutput == 0 || pid->maxIOutput > (max - min))
	{
		PID_setMaxIOutput(pid, max - min);
	}
}

void PID_setDirection(PID_Struct* pid, int reversed)
{
	pid->reversed = reversed;
}

void PID_setSetpoint(PID_Struct* pid, float setpoint)
{
	pid->setpoint = setpoint;
}

float PID_skipCycle(PID_Struct* pid)
{
	pid->prev_time = HAL_GetTick();
	return pid->lastOutput;
}

float PID_getOutput(PID_Struct* pid, float actual, float setpoint)
{
	float output = 0;
	float Poutput = 0;
	float Ioutput = 0;
	float Doutput = 0;
	float Foutput = 0;

	//Remember old errorSum for use in reverting errorSum if any limits are reached later on
	float oldErrorSum = pid->errorSum;

	pid->setpoint = setpoint;

	//Do the simple parts of the calculations
	float error = setpoint - actual;

	//If this is our first time running this  we don't actually _have_ a previous input or output.
	//For sensor, sanely assume it was exactly where it is now.
	//For last output, we can assume it's the current time-independent outputs.
	if (pid->firstRun != 0)
	{
		pid->lastActual = actual;
		pid->prevError = error;
		pid->lastOutput = Poutput + Foutput;
		pid->prev_time = HAL_GetTick();
		pid->firstRun = 0;
	}

	//Get time difference since last run
	float dt = (float)(HAL_GetTick() - pid->prev_time) / (float)FREQUENCY;

	//Only run cycle when time passed is greater than 1/Hz
	if (dt < 1.0 / pid->frequency)
		return pid->lastOutput;

	//Ramp the setpoint used for calculations if user has opted to do so
	if (pid->setpointRange != 0)
	{
		setpoint = clamp(setpoint, actual - pid->setpointRange, actual + pid->setpointRange);
	}

	//Calculate F output. Notice, this depends only on the setpoint, and not the error.
	Foutput = pid->F * setpoint;

	//Calculate P term
	Poutput = pid->P * error;


	//Calculate D Term
	//If rate of change of error is positive, then the derivative term should be positive to track
	//target setpoint, as system is lagging behind
	float error_rate = (error - pid->prevError) / dt;
	Doutput = pid->D * error_rate;

	//The Iterm is more complex. There's several things to factor in to make it easier to deal with.
	// 1. maxIoutput restricts the amount of output contributed by the Iterm.
	// 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
	// 3. prevent windup by not increasing errorSum if output is output=maxOutput
	pid->errorSum += error * dt;
	Ioutput = pid->I * pid->errorSum;
#if DYNAMIC_INTEGRATOR == 0
	//Case 2: Clamp IOutput to max allowed integral output
	if (pid->maxIOutput != 0 && !bounded(Ioutput, -pid->maxIOutput, pid->maxIOutput))
	{
		Ioutput = clamp(Ioutput, -pid->maxIOutput, pid->maxIOutput);

		//Max Ioutput reached, clamp errorSum
		pid->errorSum = oldErrorSum;
	}
#else
	//TODO: Dyamic integrator clamping
	float I_max = fmax(pid->maxOutput - Poutput,0);
	float I_min = fmin(pid->minOutput - Poutput,0);
	pid->maxIOutput = I_max;
	pid->minIOutput = I_min;

	if (!bounded(Ioutput, pid->minIOutput, pid->maxIOutput))
	{
		Ioutput = clamp(Ioutput, pid->minIOutput, pid->maxIOutput);

		//Max Ioutput reached, clamp errorSum
		pid->errorSum = oldErrorSum;
	}
#endif
	//And, finally, we can just add the terms up
	output = Foutput + Poutput + Ioutput + Doutput;

	//Restrict output to our specified output and ramp limits
	//Output decent rate should be negative
	if (pid->outputRampRate != 0 && pid->outputDescentRate != 0)
	{
		//If output is positive, allow outputRampRate increase and outputDescentRate decrease
		if(pid->lastOutput > 0)
		{
			if(!bounded(output, pid->lastOutput + pid->outputDescentRate * dt, pid->lastOutput + pid->outputRampRate * dt))
			{
				output = clamp(output, pid->lastOutput + pid->outputDescentRate * dt, pid->lastOutput + pid->outputRampRate * dt);
				pid->errorSum = oldErrorSum;
			}
		}

		else
		{
			if(!bounded(output, pid->lastOutput - pid->outputRampRate * dt, pid->lastOutput - pid->outputDescentRate * dt))
			{
				output = clamp(output, pid->lastOutput - pid->outputRampRate * dt, pid->lastOutput - pid->outputDescentRate * dt);
				pid->errorSum = oldErrorSum;
			}
		}
	}

	//Restrict output if output surpasses max and minimum values
	if (pid->minOutput != pid->maxOutput && !bounded(output, pid->minOutput, pid->maxOutput))
	{
		output = clamp(output, pid->minOutput, pid->maxOutput);

		//Prevent errorsum from increasing if max output is already capped
		pid->errorSum = oldErrorSum;
	}

	if (pid->outputFilter != 0)
	{
		output = pid->lastOutput * pid->outputFilter + output * (1 - pid->outputFilter);
	}

	pid->lastOutput = output;
	pid->prev_time = HAL_GetTick();
	pid->prevError = error;
	pid->lastActual = actual;
	return output;
}

float PID_getOutputFast(PID_Struct* pid)
{
	return PID_getOutput(pid, pid->lastActual, pid->setpoint);
}

void PID_reset(PID_Struct* pid)
{
	pid->firstRun = 1;
	pid->errorSum = 0;
}

void PID_setOutputRampRate(PID_Struct* pid, float rate)
{
	pid->outputRampRate = rate;
}

void PID_setOutputDescentRate(PID_Struct* pid, float rate)
{
	pid->outputDescentRate = rate;
}

void PID_setSetpointRange(PID_Struct* pid, float range)
{
	pid->setpointRange = range;
}

void PID_setOutputFilter(PID_Struct* pid, float strength)
{
	if(strength == 0 || bounded(strength, 0, 1))
		pid->outputFilter = strength;
}

void PID_setFrequency(PID_Struct* pid, float freq)
{
	pid->frequency = freq;
}
