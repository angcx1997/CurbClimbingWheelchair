/*
 * differentialDrive.h
 *
 *  Created on: 26 Nov 2021
 *      Author: ray
 *      Convert joystick data to velocity output
 */

#ifndef INC_DIFFERENTIALDRIVE_H_
#define INC_DIFFERENTIALDRIVE_H_

#define COMPUTERANGE 18000


#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "stm32f4xx.h"
//user need to input the following

typedef enum{
	GEAR1 = 40,
	GEAR2 = 60,
	GEAR3 = 80,
	GEAR4 = 100,
}Gear_Level;

/*
 * User can configure based on requirement
 */
typedef struct{
	float min_vel; 			/*!< Motor minimum velocity >*/
	float max_vel; 			/*!< Motor maximum speed >*/
	float min_acc; 			/*!< Motor minimum acceleration >*/
	float max_acc;			/*!< Motor maximum acceleration >*/
	float min_jerk; 		/*!< Motor minimum jerk >*/
	float max_jerk;			/*!< Motor maximum jerk >*/
}speedConfig;

/*
 * Use to store motor velocity state to calculate acceleration
 */
typedef struct{
	float vel;
	float prev_vel;
	uint32_t prev_t;
}motorState;

typedef struct{
	float m_leftMotor;	/*!< Motor (left)  mixed output (-1..+1) >*/
	float m_rightMotor;	/*!< Motor (right) mixed output (-1..+1) >*/
	motorState left_motor_state;	/*!< Motor (left) state >*/
	motorState right_motor_state;	/*!< Motor (right) state >*/
	uint32_t frequency; 			/*!< To smoothen output >*/
}differentialDrive_Handler;

//User configurable output speed characteristic
extern speedConfig speed_config;

/*********************************************************************
 * @fn      		  - DDrive_Init
 *
 * @brief             - Used to initialize the differential drive handler member to 0
 *
 * @param[differentialDrive_Handler*] - pointer to handler. User must initialize the handler in the main file
 *
 * @return            - None
 *
 * @Note              -
 */
void DDrive_Init(differentialDrive_Handler* dd_handler, uint32_t frequency);

/*********************************************************************
 * @fn      		  - DDrive_SpeedConfiguration
 *
 * @brief             - Used to initialize the speed configuration member
 *
 * @param[differentialDrive_Handler*] - pointer to handler. User must initialize the handler in the main file
 *
 * @return            - None
 *
 * @Note              -
 */
void DDrive_SpeedConfiguration(speedConfig* speed_config);

/*********************************************************************
 * @fn      		  - DDrive_SpeedMapping
 *
 * @brief             - Compute differential steering from (x,y) values.
 * 						Compute speed of the output motor through mapping of
 * 						joystick position to relative speed
 *
 * @param[XValue]     - X value in [-MAX_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE] range.
 * @param[YValue]     - YValue: Y value in [-MAX_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE] range.
 *
 * @return            - None
 *
 * @Note              -
 */
void DDrive_SpeedMapping(differentialDrive_Handler* dd_handler, int XValue, int YValue, Gear_Level gear_level);

/*********************************************************************
 * @fn      		  - DDrive_SpeedLimiter
 *
 * @brief             - Ramp up speed to smoothen trajectory
 * @param[motor_state]  - Pointer to motor state.
 * @param[frequency]    - Frequency of function being call to calculate acceleration.
 * @param[velocity]     - Desired velocity.
 * @return            - None
 *
 * @Note              -
 */
void DDrive_SpeedLimiter(motorState* motor_state, uint32_t frequency, float velocity);

extern differentialDrive_Handler differential_drive_handler;


#endif /* INC_DIFFERENTIALDRIVE_H_ */
