/*
 * differentialDrive.c
 *
 *
 *  Created on: 26 Nov 2021
 *      Author: ray
 */

#include "differentialDrive.h"


#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#define MIN(x,y) (((x) < (y)) ? (x) : (y))

#define MAPPING_ALGORITHM 2

//Local function prototype declaration
static void limit_velocity(float* v);
static void limit_acceleration(float* v, float prev_v, float dt);
static float clamp(float x, float min, float max);

void DDrive_Init(differentialDrive_Handler* dd_handler, uint32_t frequency){
	dd_handler->m_leftMotor = 0;
	dd_handler->m_rightMotor = 0;
	dd_handler->m_fPivYLimit = 0.15 * COMPUTERANGE;
	dd_handler->frequency = frequency;
	memset(&dd_handler->left_motor_state, 0, sizeof(dd_handler->left_motor_state));
	memset(&dd_handler->right_motor_state, 0, sizeof(dd_handler->right_motor_state));
}

void DDrive_SpeedConfiguration(speedConfig* speed_config){
	speed_config->min_vel = 0;
	speed_config->max_vel = 0;
	speed_config->min_acc = 0;
	speed_config->max_acc = 0;
	speed_config->min_jerk = 0;
	speed_config->max_jerk = 0;
}

void DDrive_SpeedMapping(differentialDrive_Handler* dd_handler, int XValue, int YValue, Gear_Level gear_level) {

#if MAPPING_ALGORITHM == 1
	float   nMotPremixL = 0;    // Motor (left)  premixed output        (-MAX_JOYSTICK_VALUE..+MAX_JOYSTICK_VALUE)
	float   nMotPremixR = 0;    // Motor (right) premixed output        (-MAX_JOYSTICK_VALUE..+MAX_JOYSTICK_VALUE)
	int     nPivSpeed = 0;      // Pivot Speed                          (-MAX_JOYSTICK_VALUE..+MAX_JOYSTICK_VALUE)
	float   fPivScale = 0;      // Balance scale b/w drive and pivot    (   0..1   )
    // Calculate Drive Turn output due to Joystick X input
    if (YValue >= 0) {
        // Forward
        nMotPremixL = (XValue >= 0) ? COMPUTERANGE : (COMPUTERANGE + XValue);
        nMotPremixR = (XValue >= 0) ? (COMPUTERANGE - XValue) : COMPUTERANGE;
    } else {
        // Reverse
        nMotPremixL = (XValue >= 0) ? (COMPUTERANGE - XValue) : COMPUTERANGE;
        nMotPremixR = (XValue >= 0) ? COMPUTERANGE : (COMPUTERANGE + XValue);
    }

    // Scale Drive output due to Joystick Y input (throttle)
    nMotPremixL = nMotPremixL * YValue / COMPUTERANGE;
    nMotPremixR = nMotPremixR * YValue / COMPUTERANGE;

    // Now calculate pivot amount
    // - Strength of pivot (nPivSpeed) based on Joystick X input
    // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
    nPivSpeed = (abs(YValue) > dd_handler->m_fPivYLimit) ? XValue : XValue * 0.50;
    fPivScale = (abs(YValue) > dd_handler->m_fPivYLimit) ? 0.0 : (1.0 - (float)(abs(YValue)) / (float)dd_handler->m_fPivYLimit);

    // Calculate final mix of Drive and Pivot
    nMotPremixL  = (1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed);
    nMotPremixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

    //Normalize the output from -1 to 1
	dd_handler->m_leftMotor = (nMotPremixL / (float)COMPUTERANGE) * ((float)gear_level/100.0);
	dd_handler->m_rightMotor = (nMotPremixR / (float)COMPUTERANGE) * ((float)gear_level/100.0);
#elif  MAPPING_ALGORITHM == 2
	float x = ((float)XValue / COMPUTERANGE);
	float y = ((float)YValue / COMPUTERANGE);

	//Convert input into exponential form for finer control at crowded area
	x = x * x * x;
	y = y * y * y;

	if (0.5 * x > 0.1)
		x = 0.1 + (0.5 * x - 0.1) * 0.5;
	else if (0.5 * x < -0.1)
		x = -0.1 + (0.5 * x + 0.1) * 0.5;
	else
		x = 0.5 * x;
	float tmp_r = y-x;
	float tmp_l = y+x;
	dd_handler->m_leftMotor = tmp_l * ((float)gear_level/100.0);
	dd_handler->m_rightMotor = tmp_r * ((float)gear_level/100.0);

#endif
	//Force motor when speed approximately zero to avoid current consumption
	dd_handler->m_leftMotor = 	(dd_handler->m_leftMotor < 0.005 && dd_handler->m_leftMotor > -0.005) ? 0 : dd_handler->m_leftMotor;
	dd_handler->m_rightMotor = 	(dd_handler->m_rightMotor < 0.005 && dd_handler->m_rightMotor > -0.005)? 0 : dd_handler->m_rightMotor;
}

void DDrive_SpeedLimiter(motorState* motor_state, uint32_t frequency, float velocity){
	float dt;
	//If first read or stationary for too long
	if (motor_state->prev_t == 0){
		motor_state->prev_t = HAL_GetTick();
		motor_state->prev_vel = velocity;
		return;
	}
	motor_state->vel = velocity;
	dt = (float)(HAL_GetTick() - motor_state->prev_t) / (float)frequency;

	limit_acceleration(&motor_state->vel, motor_state->prev_vel, dt);
	limit_velocity(&motor_state->vel);
	motor_state->prev_vel = motor_state->vel;
	motor_state->prev_t = HAL_GetTick();
}

/*********************************************************************
 * @fn      		- limit_velocity
 *
 * @brief          	- Clamp velocity between speed define in speed_config
 *
 * @param[*v]     	- latest velocity calculated
 * @param[*speed_config] - user defined min/max desired speed configuration
 *
 * @return            - None
 *
 * @Note              -
 */
static void limit_velocity(float* v)
{
    *v = clamp(*v, speed_config.min_vel, speed_config.max_vel);
}

/*********************************************************************
 * @fn      		- limit_acceleration
 *
 * @brief          	- Clamp acceleration between acceleration define in speed_config
 *
 * @param[*v]     		- latest velocity calculated
 * @param[prev_v]     	- previous velocity
 * @param[dt]     		- time elapsed
 * @param[*speed_config]- user defined min/max desired speed configuration
 *
 * @return            - None
 *
 * @Note              -
 */
static void limit_acceleration(float *v, float prev_v, float dt)
{
    float dv_min = 0;
    float dv_max = 0;
    float dv = 0;
    if (prev_v >= 0){
    	dv_min = speed_config.min_acc * dt;
    	dv_max = speed_config.max_acc * dt;
    	dv = clamp(*v - prev_v, dv_min, dv_max);
    }
    else{
    	//Reverse use slower acceleration as the speed is only half the forward speed
    	//Use high accel and decel would cause jerk when stop during reverse
    	//Note that the sign need to change when reverse
    	dv_min = (speed_config.min_acc*0.5) * dt;
		dv_max = (speed_config.max_acc*0.5) * dt;
    	dv = clamp(*v - prev_v,-dv_max , -dv_min);
    }

    *v = prev_v + dv;
}

/*********************************************************************
 * @fn      		- clamp
 *
 * @brief          	- Clamp x between min and max
 *
 * @param[x]     		- input value
 * @param[min]     		- min value allowed
 * @param[max]     		- max value allowed
 *
 * @return            - None
 *
 * @Note              -
 */
static float clamp(float x, float min, float max)
{
  return MIN(MAX(min, x), max);
}



