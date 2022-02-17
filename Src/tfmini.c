/*
 * tfmini.c
 *
 *  Created on: Feb 14, 2022
 *      Author: ray
 */


#include "tfmini.h"

#include "math.h"

#include "usbd_cdc_if.h"
#include "usb_device.h"

void USB_TransmitData(uint8_t command)
{
	CDC_Transmit_FS((uint8_t*)&command, 1);
}

int prev_dist = 0;//Calculate curb difference
uint16_t prev_dis = 0; //EMA

uint16_t TFMINI_Plus_RcvData(uint8_t *pBuffer, uint8_t length)
{
	uint8_t i = 0;
	uint16_t checkSum = 0;

	if(length == 	TFMINI_SIZE){
		if((pBuffer[0] == TFMINI_HEADER) && (pBuffer[1] == TFMINI_HEADER)){
			for(i=0; i < TFMINI_SIZE-1; i++){
				checkSum += pBuffer[i];
			}
			if(pBuffer[TFMINI_SIZE-1] == (checkSum & 0x00ff)){
				uint16_t distance = pBuffer[2] | (pBuffer[3] << 8);
				uint16_t strength = pBuffer[4] | (pBuffer[5] << 8);
				if(strength <100 || strength == 65535)
					return 0;
				return distance;
			}else{
				return 0;
			}
		}else{
			return 0;
		}
	}else{
		return 0;
	}

}


int detectCurb_down(uint16_t distance)
{
	int diff = 0;

	if(prev_dist == 0){
		prev_dist = distance;
		return 0;
	}

	diff =(int)distance - prev_dist;
	prev_dist = (int)distance;

	return diff;
}

uint16_t Noise_loop(uint16_t distance)
{

	if(prev_dis == 0){
			prev_dis = distance;
			return 0;
		}

	uint16_t newDis = prev_dis*ALPHA + distance*(1-ALPHA);
	prev_dis = distance;
	return newDis;
}
/*
 float EMA_Function(float alpha, uint16_t latest, uint16_t stored)
{
	return (alpha*latest + (1.0f-alpha)*stored);

}
*/
