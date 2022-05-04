/*
 * DataLogger.c
 *
 *  Created on: Apr 29, 2022
 *      Author: angcx
 */

#include "DataLogger.h"

#include <stdio.h>
#include <stdlib.h>

/**
 * Byte length
 */
#define SOI_BYTE 1
#define EOI_BYTE 1
#define CHECKSUM_BYTE 2
#define SIZE_BYTE 1
#define CONFIG_BYTE_LENGTH (SOI_BYTE+EOI_BYTE+CHECKSUM_BYTE+SIZE_BYTE)

/**
 * Byte Initial and ending byte
 */
#define DATA_LOGGER_SOI 0xFF
#define DATA_LOGGER_EOI	0xFB

#define INFO_START_IDX	2
static uint8_t data_logger_array[50];
static uint16_t CheckSum(uint8_t arr[], uint16_t Size);

void DataLogger_Init(DataLogger_Msg_t* msg){
	msg->arr_ptr_idx = INFO_START_IDX;
	msg->pMsg = data_logger_array;
	msg->pMsg[0] = DATA_LOGGER_SOI;
}

void DataLogger_AddMessage(DataLogger_Msg_t* msg, uint8_t arr[], uint16_t Size)
{
	for(int i = 0; i < Size; i++)
		msg->pMsg[msg->arr_ptr_idx + i] = arr[i];
	msg->arr_ptr_idx += Size;
}

void DataLogger_CompleteTxMessage(DataLogger_Msg_t* msg){
	msg->size = msg->arr_ptr_idx - INFO_START_IDX + CONFIG_BYTE_LENGTH;
	//Size
	msg->pMsg[1] = msg->arr_ptr_idx - INFO_START_IDX;
	//Checksum
	uint16_t checksum = CheckSum(msg->pMsg,msg->size - CHECKSUM_BYTE - EOI_BYTE);
	msg->pMsg[msg->arr_ptr_idx++] = checksum>>8 & 0xFF;
	msg->pMsg[msg->arr_ptr_idx++] = checksum>>0 & 0xFF;
	//EOI
	msg->pMsg[msg->size - 1] = DATA_LOGGER_EOI;
	//Reset pointer index
	msg->arr_ptr_idx = 2;
}


static uint16_t CheckSum(uint8_t arr[], uint16_t Size){
	uint16_t checksum;
	for(int i = 0; i < Size;i++){
		checksum += (uint16_t)(arr[i]);
	}
	return checksum;
}


