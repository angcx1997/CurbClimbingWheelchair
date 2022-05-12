/*
 * us_delay.h
 *
 *  Created on: 12 May 2022
 *      Author: ray
 *
 *
 */

#ifndef US_DELAY_H_
#define US_DELAY_H_

#include "stdint.h"
#include "stm32f4xx.h"

/**
 * Initialize the local timer pointer to user specific timer
 * @param htim pointer to timer for microsecond delay
 * @note Please configure the timer such that each increment is equivalent to 1us.
 * @example If clock rate is 72MHz, set prescaler as (72-1) to achieve 1Mhz.
 * 		which is equivalent to 1Mhz(1us)
 */
void delay_us_init(TIM_HandleTypeDef* htim);

/**
 * Used for microsecond delay
 * @param us number of us to wait
 */
void delay_us (uint16_t us);

#endif /* US_DELAY_H_ */
