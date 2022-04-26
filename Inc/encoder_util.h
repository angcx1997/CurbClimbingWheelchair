/*
 * encoder.util.h
 *
 *  Created on: 26 Apr 2022
 *      Author: ray
 */

#ifndef ENCODER_UTIL_H_
#define ENCODER_UTIL_H_

#include "stm32f4xx.h"
#include "briter_encoder_rs485.h"
#include "stdint.h"

/**
 * Store wheel position and velocity
 */
typedef struct{
    uint32_t	last_position;		/*!<Last position of encoder*/
    uint32_t	curr_position;		/*!<Current position of encoder*/
    int32_t	d_position;		/*!<Position travelled during d_tick*/
    int32_t	total_position;		/*!<Position with direction*/
    float 	velocity;		/*!<Velocity of encoder*/
    uint32_t	last_tick;		/*!<Last time function get called*/
}wheel_velocity_t;

/**
 * @brief Calculate velocity of wheel through encoder and update wheel_velocity_t
 * @param wheel pointer to wheel velocity struct
 * @param curr_position latest encoder reading
 * @retval None
 */
void calculateVelocity(wheel_velocity_t* wheel, uint32_t curr_position);




#endif /* ENCODER_UTIL_H_ */
