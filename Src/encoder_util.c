/*
 * encoder_util.c
 *
 *  Created on: 26 Apr 2022
 *      Author: ray
 */

#include "encoder_util.h"
#include <math.h>



float calculateVelocity(wheel_velocity_t* wheel, uint32_t curr_position){
    //If first iteration
    if(wheel->last_position == 0 && wheel->last_tick == 0){
	wheel->last_position = curr_position;
	wheel->last_tick = HAL_GetTick();
	return 0.0;
    }
    //Period between two tick in ms
    uint8_t tick_period = HAL_GetTickFreq();
    //Time elapsed in ms
    uint32_t dt = (HAL_GetTick() - wheel->last_tick) * tick_period;
    //Calculate change in position and added to total position travel
    wheel->d_position = curr_position - wheel->last_position;

    //Encoders will wrap around, offset the wrap around if it does happen
    //Wrap around is detected by  checking if the difference in encoder value exceeds half the max encoder value
    if (wheel->d_position < -BRITER_RS485_MAX_VALUE * 0.5)
	wheel->d_position += BRITER_RS485_MAX_VALUE;
    else if(wheel->d_position > BRITER_RS485_MAX_VALUE * 0.5)
	wheel->d_position -= BRITER_RS485_MAX_VALUE;

    wheel->total_position += wheel->d_position;

    //Calculate velocity
    wheel->velocity = WHEEL_RADIUS * 1000.0 * (float)(wheel->d_position) / BRITER_RS485_PPR  * 2.0 * 3.1415926 / (float)dt;

    if (fabs(wheel->velocity) < 0.05)
	wheel->velocity = 0;
    //Store last reading for subsequent calculation
    wheel->last_tick = HAL_GetTick();
    wheel->last_position = curr_position;

    return wheel->velocity;
}


