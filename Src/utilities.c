/*
 * utilities.c
 *
 *  Created on: Mar 15, 2022
 *      Author: ray
 */

#include "utilities.h"

uint16_t CalculateChecksum_16bit(uint8_t *arr, uint16_t size)
{
    uint16_t checksum = 0;
    for(int i = 0; i < size; i++){
	checksum += arr[i];
    }
    return checksum;
}

uint8_t CalculateChecksum_8bit(uint8_t *arr, uint16_t size)
{
    uint8_t checksum = 0;
    for(int i = 0; i < size; i++){
	checksum += arr[i];
    }
    return checksum;
}

bool ValidateChecksum_16bit(uint8_t *arr, uint16_t size)
{
    uint16_t checksum = CalculateChecksum_16bit(arr, size-2);
    if (arr[size-2] == (uint8_t) (checksum & 0xff) &&
	    arr[size-1] == (uint8_t) ((checksum >> 8) & 0xff))
	return true;
    return false;
}


bool ValidateChecksum_8bit(uint8_t *arr, uint16_t size)
{
    uint8_t checksum = CalculateChecksum_8bit(arr, size-1);
    if (arr[size-1] == checksum)
	return true;
    return false;
}
