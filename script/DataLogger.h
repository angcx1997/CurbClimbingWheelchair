/*
 * DataLogger.h
 *
 *  Created on: Apr 29, 2022
 *      Author: angcx
 *
 * Message
 * 1. SOI (1 byte)
 * 2. Data size (1 byte)
 * 3. Info (number of byte based on data size)
 * 4. Checksum (2 byte)
 * 5. EOI (1 byte)
 */

#ifndef DATALOGGER_H_
#define DATALOGGER_H_

#include <stdint.h>
#include <stdbool.h>
#include <UnionDataStruct.h>

/**
 * Stack allocated to store array that want to be send
 * Example: if user want to send 100 byte, put >=100
 */
#define DATA_LOGGER_STACK_ALLOCATED			50

/**
 * Data logger transmit message type
 */
typedef struct{
	uint8_t* pMsg;			/*!<Pointer to the array storing the tx message*/
	uint16_t size;			/*!<Size of the tx message*/
	uint16_t arr_ptr_idx;	/*!<array pointer index*/
}DataLogger_Msg_t;

/**
 * @brief use to Init Tx Message
 * @param msg pointer to datalogger
 * @param size total number of byte user intend to send, exclude soi,eoi,checksum
 */
void DataLogger_Init(DataLogger_Msg_t* msg);

/**
 * @brief Add message to data_logger
 * @param msg pointer to datalogger
 * @param arr pointer to user added array
 * @param size size of user addded array
 */
void DataLogger_AddMessage(DataLogger_Msg_t* msg, uint8_t arr[], uint16_t Size);


/**
 * @brief use to construct Tx Message
 * @param msg pointer to datalogger
 */
void DataLogger_CompleteTxMessage(DataLogger_Msg_t* msg);




#endif /* DATALOGGER_H_ */
