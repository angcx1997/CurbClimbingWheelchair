/*
 * tfmini.h
 *
 *  Created on: 14 Feb 2022
 *      Author: ray
 */

#ifndef INC_TFMINI_H_
#define INC_TFMINI_H_

#include "stdint.h"

#define TFMINI_HEADER    0x59
#define TFMINI_SIZE      9
#define TFMINI_RX_SIZE          9

#define CURBHEIGHT       0xF

#define ALPHA    0.20

float EMA_Function(float alpha, uint16_t latest, uint16_t stored);

uint16_t Noise_loop(uint16_t distance);

uint16_t detectCurb_down(uint16_t distance);

uint16_t TFMINI_Plus_RcvData(uint8_t *pBuffer, uint8_t length);

enum{
	CURB_CHANGE = 0x01,
	USB_STOP = 0x02,
	USB_MOVE = 0x03,
	USB_CLIMB_UP = 0x04,
	USB_CLIMB_DOWN = 0x05
};

void USB_TransmitData(uint8_t command);


#endif /* INC_TFMINI_H_ */
