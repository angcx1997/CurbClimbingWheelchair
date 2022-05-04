/*
 * UnionDataStruct.h
 *
 *  Created on: Apr 29, 2022
 *      Author: angcx
 */

#ifndef UNIONDATASTRUCT_H_
#define UNIONDATASTRUCT_H_

#include "stdint.h"

typedef union{
	float data;
	uint8_t array[4];
}float_byte_u;

typedef union{
	int8_t data;
	uint8_t array;
}int8_byte_u;

typedef union{
	int16_t data;
	uint8_t array[2];
}int16_byte_u;

typedef union{
	int32_t data;
	uint8_t array[4];
}int32_byte_u;

typedef union{
	uint16_t data;
	uint8_t array[2];
}uint16_byte_u;

typedef union{
	uint32_t data;
	uint8_t array[4];
}uint32_byte_u;

typedef union{
	uint8_t data;
	uint8_t array;
}uint8_byte_u;



#endif /* UNIONDATASTRUCT_H_ */
