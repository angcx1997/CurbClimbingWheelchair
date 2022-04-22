/** @file   common.h
 *  @brief  Header file of commonly used macros and functions.
 *  @author Rehabilitation Research Institute of Singapore / MRBA Team
 */

#ifndef   	MRBA3_COMMON_H
#define		MRBA3_COMMON_H

#include "stm32f4xx_hal.h"


typedef struct
{
	GPIO_TypeDef* 	gpioPort;
	uint16_t	gpioPin;
	uint8_t		curRead;		/*!< Use to store and compare GPIO reading >*/
	uint8_t		preRead;		/*!< Use to store and compare GPIO reading >*/
	uint8_t   	state;			/*!< GPIO state by filtering the debounce >*/
	uint32_t	lastDebounceTime; 	/*!< Use to store moment button pressed >*/
}Button_TypeDef;

/*
 * brief Read GPIO filter input
 * param hgpio 			pointer to gpio handler
 * param debounce_time	button time to finish debouncing
 */
uint8_t Button_FilteredInput(Button_TypeDef* hgpio, uint32_t debounce_time);

#endif
