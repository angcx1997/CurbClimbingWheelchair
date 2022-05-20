/*
 * pid.h
 *
 *  Created on: Dec 3, 2020
 *      Author: by
 */

#ifndef __PID_BASE_H
#define __PID_BASE_H
#include <main.h>

typedef struct PID{
	//Controller parameters
	float P;
	float I;
	float D;
	float F;

	//Limits
	float maxIOutput;
	float minIOutput;
	float maxError;
	float errorSum;
	float prevError;
	float deadTime;
	float frequency;

	float maxOutput;
	float minOutput;

	float setpoint;

	float lastActual;

	//Flags
	int firstRun;
	int reversed;

	//Ramping and descent limits
	float outputRampRate;
	float outputDescentRate;
	float lastOutput;

	float outputFilter;

	float setpointRange;

	uint32_t prev_time;
} PID_Struct;

void PID_Init(PID_Struct* pid);
void checkSigns(PID_Struct* pid);
float clamp(float value, float min, float max);
uint8_t bounded(float value, float min, float max);
void PID_setPID(PID_Struct* pid, float p, float i, float d);
void PID_setPIDF(PID_Struct* pid, float p, float i, float d, float f);
void PID_setF(PID_Struct* pid, float f);
void PID_setMaxIOutput(PID_Struct* pid, float maximum);
void PID_setMinIOutput(PID_Struct* pid, float minimum);
void PID_setOutputLimits(PID_Struct* pid, float min, float max);
void PID_setDirection(PID_Struct* pid, int reversed);
void PID_setSetpoint(PID_Struct* pid, float setpoint);
float PID_skipCycle(PID_Struct* pid);
float PID_getOutput(PID_Struct* pid, float actual, float setpoint);
float PID_getOutputFast(PID_Struct* pid);
void PID_reset(PID_Struct* pid);
void PID_setOutputRampRate(PID_Struct* pid, float rate);
void PID_setOutputDescentRate(PID_Struct* pid, float rate);
void PID_setSetpointRange(PID_Struct* pid, float range);
void PID_setOutputFilter(PID_Struct* pid, float strength);
void PID_setFrequency(PID_Struct* pid, float freq);

#endif /* INC_PID_H_ */
























