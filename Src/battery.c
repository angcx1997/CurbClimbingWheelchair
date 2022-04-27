/*
 * battery.c
 *
 *  Created on: 30 Nov 2021
 *      Author: ray
 */


#include "battery.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/*
 * Send byte format
 */
#define SOI 			0xDD	/*!< Indicate start of transmission >*/
#define EOI 			0x77	/*!< Indicate start of transmission >*/
#define BATTERY_READ 	0xA5	/*!< Read battery state >*/
#define BATTERY_WRITE 	0x5A	/*!< Write battery state >*/

/*
 * Send array index
 */
#define SEND_COMMAND_IDX 	2
#define SEND_LENGTH_IDX		3

/*
 * Reply array index
 */
#define REPLY_FLAG_IDX 		2
#define REPLY_LENGTH_IDX 	3
#define REPLY_INFO_IDX		4

enum Command{				/*!< Indicate what reply want from hardware >*/
	BATTERY_BASIC_STATE = 0x03,
	SINGLE_BATTERY_VOLTAGE,
	BATTERY_SERIAL_NUMBER
};

/*
 * brief Calculate checksum from specific range
 * param buf		array pointer
 * param start_pos 	initial position to start calculation
 * param end_pos 	ending position for calculation
 */
static uint16_t CalculateCheckSum(uint8_t buf[], int start_pos, int end_pos);

void Battery_Init(batteryHandler* battery_handler, UART_HandleTypeDef* huart){
	battery_handler->huart = huart;
	memset(&battery_handler->battery_info, 0, sizeof(battery_handler->battery_info));
	battery_handler->battery_info.NTC_content = NULL;
}

void Battery_DeInit(batteryHandler* battery_handler){
	free(battery_handler->huart);
	free(battery_handler->battery_info.NTC_content);
}

HAL_StatusTypeDef Battery_GetState(batteryHandler* battery_handler){
	HAL_StatusTypeDef status;
	uint8_t send_buf[7] = {0};
	send_buf[0] = SOI;
	send_buf[1] = BATTERY_READ;
	send_buf[2] = BATTERY_BASIC_STATE;
	send_buf[3] = 0x00; //Length of info desired, 0 if want to get all info
	uint16_t checksum = CalculateCheckSum(send_buf, SEND_COMMAND_IDX, SEND_LENGTH_IDX);
	send_buf[4] = (checksum >> 8) & 0xFF;
	send_buf[5] = checksum & 0xFF;
	send_buf[6] = EOI;
	status = HAL_UART_Transmit(battery_handler->huart, send_buf, sizeof(send_buf), 10);
	if (status != HAL_OK)
		return status;

	if (__HAL_UART_GET_FLAG(battery_handler->huart, UART_FLAG_ORE)){
	    __HAL_UART_CLEAR_OREFLAG(battery_handler->huart);
	}


//	status = HAL_UARTEx_ReceiveToIdle_IT(battery_handler->huart, battery_rx_buf, sizeof(battery_rx_buf));
////	if (status != HAL_OK)
////		return status;
//	return status;

	uint8_t receive_buf[40];
	uint16_t rx_length;
	status = HAL_UARTEx_ReceiveToIdle(battery_handler->huart, receive_buf, sizeof(receive_buf), &rx_length, 50);
	return Battery_ReadState(battery_handler, receive_buf);
}

HAL_StatusTypeDef Battery_ReadState(batteryHandler* battery_handler, uint8_t battery_rx_buf[]){
	//Last index number in buffer
	uint8_t end_idx = REPLY_INFO_IDX + battery_rx_buf[REPLY_LENGTH_IDX] + 2;
	//check receive start, r/w state. flag and end buffer
	if (	battery_rx_buf[0] != SOI ||
			battery_rx_buf[end_idx] != EOI ||
			(battery_rx_buf[1] != BATTERY_BASIC_STATE)
			)
		return HAL_ERROR;

	//Checksum_end_idx need to consider the length received from reply,
	uint8_t checksum_end_idx = REPLY_INFO_IDX + battery_rx_buf[REPLY_LENGTH_IDX] - 1;
	uint16_t checksum = CalculateCheckSum(battery_rx_buf, REPLY_FLAG_IDX, checksum_end_idx);
	//Check the last second and third byte, if doesnt match the check sum, dispose the data
	if (	battery_rx_buf[REPLY_INFO_IDX + battery_rx_buf[REPLY_LENGTH_IDX] 	  ] != ((checksum >> 8) & 0xFF) ||
			battery_rx_buf[REPLY_INFO_IDX + battery_rx_buf[REPLY_LENGTH_IDX] +1 ] != (checksum & 0xFF)	)
			return HAL_ERROR;

	uint8_t i = 4;
	battery_handler->battery_info.total_voltage = (battery_rx_buf[i++] << 8) | battery_rx_buf[i++];
	battery_handler->battery_info.current = (battery_rx_buf[i++] << 8) | battery_rx_buf[i++];
	battery_handler->battery_info.remaining_capacity = (battery_rx_buf[i++] << 8) | battery_rx_buf[i++];
	battery_handler->battery_info.nominal_capacity = (battery_rx_buf[i++] << 8) | battery_rx_buf[i++];
	battery_handler->battery_info.cycles = (battery_rx_buf[i++] << 8) | battery_rx_buf[i++];
	battery_handler->battery_info.production_date = (battery_rx_buf[i++] << 8) | battery_rx_buf[i++];
	battery_handler->battery_info.balanced_state[0] = (battery_rx_buf[i++] << 8) | battery_rx_buf[i++];
	battery_handler->battery_info.balanced_state[1] = (battery_rx_buf[i++] << 8) | battery_rx_buf[i++];
	battery_handler->battery_info.protection_state = (battery_rx_buf[i++] << 8) | battery_rx_buf[i++];
	battery_handler->battery_info.software_version = battery_rx_buf[i++];
	battery_handler->battery_info.remaining_capacity_RSOC = battery_rx_buf[i++];
	battery_handler->battery_info.FET_control_status = battery_rx_buf[i++];
	battery_handler->battery_info.battery_number = battery_rx_buf[i++];
	//Check if NTC number is the same
	//For first iteration
	if (battery_handler->battery_info.NTC_number == 0){
		battery_handler->battery_info.NTC_number = battery_rx_buf[i];
		battery_handler->battery_info.NTC_content = (uint16_t*)malloc(battery_handler->battery_info.NTC_number*sizeof(uint16_t));
	}
	//for second onward iteration
	//Check if NTC number is the same, do nothing
	else if (battery_handler->battery_info.NTC_number == battery_rx_buf[i]){
		battery_handler->battery_info.NTC_number = battery_rx_buf[i];
	}
	//Check if NTC is different
	//realloc memory to change the size of array
	else if (battery_handler->battery_info.NTC_number != battery_rx_buf[i]){
		if(battery_handler->battery_info.NTC_content != NULL){
			battery_handler->battery_info.NTC_number = battery_rx_buf[i];
			battery_handler->battery_info.NTC_content = realloc(battery_handler->battery_info.NTC_content, battery_handler->battery_info.NTC_number*sizeof(uint16_t));
		}
	}
	for(int j = 0; j <= battery_handler->battery_info.NTC_number; j++){
		battery_handler->battery_info.NTC_content[j] =  (battery_rx_buf[i+2*j+1] << 8) & 0xFF00;
		battery_handler->battery_info.NTC_content[j] |=  battery_rx_buf[i+2*j+2] & 0x00FF;
	}
	return HAL_OK;
}

uint16_t CalculateCheckSum(uint8_t buf[], int start_pos, int end_pos){
	uint16_t checksum = 0;
	for(int i = start_pos; i <= end_pos; i++){
		checksum += (uint16_t)buf[i];
	}
	//Formula given by datasheet
	checksum = (0xFFFF ^ checksum) + 1;
	return checksum;
}


