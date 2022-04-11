/*
 * briter_encoder_rs485.c
 *
 *  Created on: 11 Apr 2022
 *      Author: ray
 */

#include "briter_encoder_rs485.h"
#include <string.h>

/** @defgroup briter_encoder_rs485 function type
 * @{
 */
typedef enum {
    ENC_READ = 0x03, /*!< Read holding register*/
    ENC_WRITE_SINGLE = 0x06, /*!< Write to single register*/
    ENC_WRITE_MULTI = 0x10, /*!< Write to multiple register*/
} RS485_Enc_Func_t;
/**
 * @}
 */

/** @defgroup briter_encoder_rs485 transmit/receive data type
 * @{
 */
/** Used when host sending data to slave via READ or WRITE_SINGLE*/
typedef union {
    struct send_info {
	uint8_t address; /*!< Slave address*/
	uint8_t function; /*!< Refer to briter_encoder_rs485 function type*/
	uint8_t start_register[2];
	uint8_t register_number[2]; /*!< Number of register want to read*/
	uint8_t crc[2]; /*!< CRC from 0 to 6*/
    } send_info;
    uint8_t buf[8];
} Encoder_TX_t;
/**
 * @}
 */

/** @defgroup briter_encoder_rs485 Private Functions
 * @{
 */
static uint16_t Calculate_CRC(uint8_t pbuf[], uint16_t num);
static HAL_StatusTypeDef Encoder_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
static HAL_StatusTypeDef Encoder_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
static HAL_StatusTypeDef Encoder_CheckRX(uint8_t *pData,uint8_t address,RS485_Enc_Func_t func);
/**
 * @}
 */

HAL_StatusTypeDef BRITER_RS485_Init(Briter_Encoder_t *handler, uint8_t address, UART_HandleTypeDef *huart) {
    //Check if parameter is NULL ptr
    if (!handler || !huart || address == 0)
	return HAL_ERROR;
    memset(handler, 0, sizeof(Briter_Encoder_t));
    handler->addr = address;
    handler->huart = huart;
    return HAL_OK;
}

HAL_StatusTypeDef BRITER_RS485_GetEncoderValue(Briter_Encoder_t *handler) {
    //Send encoder data
    Encoder_TX_t send_t;
    memset(&send_t, 0, sizeof(send_t));
    send_t.send_info.address = (uint8_t) (handler->addr);
    send_t.send_info.function = ENC_READ;
    send_t.send_info.start_register[0] = (uint8_t) ((BRITER_RS485_VALUE_ADDR >> 8) & 0xFF);
    send_t.send_info.start_register[1] = (uint8_t) ((BRITER_RS485_VALUE_ADDR >> 0) & 0xFF);
    uint16_t register_number = 2;
    send_t.send_info.register_number[0] = (uint8_t) ((register_number >> 8) & 0xFF);
    send_t.send_info.register_number[1] = (uint8_t) ((register_number >> 0) & 0xFF);
    uint16_t crc = Calculate_CRC(send_t.buf, sizeof(send_t.buf) - 2);
    send_t.send_info.crc[0] = (uint8_t) ((crc >> 8) & 0xFF);
    send_t.send_info.crc[1] = (uint8_t) ((crc >> 0) & 0xFF);
    if (Encoder_Transmit(handler->huart, send_t.buf, sizeof(send_t.buf)) != HAL_OK)
	return HAL_ERROR;

    //Receive return from slave
    uint8_t receive_buf[9];
    if (Encoder_Receive(handler->huart, receive_buf, sizeof(receive_buf)) != HAL_OK)
	return HAL_ERROR;

    //Check receive buffer
    if (Encoder_CheckRX(receive_buf, (uint8_t) (handler->addr), ENC_READ) != HAL_OK)
	return HAL_ERROR;

    return Encoder_CheckRX();
}

/**
 * @brief  Calculate_CRC.
 * @param  pbuf pointer to buffer
 * @param  pbuf size of buffer that you wish to included in CRC calculation
 * @retval CRC value
 */
static uint16_t Calculate_CRC(uint8_t pbuf[], uint16_t num) {
uint8_t i, j;
uint16_t wcrc = 0xffff;
for (i = 0; i < num; i++) {
    wcrc ^= (uint16_t) (pbuf[i]);
    for (j = 0; j < 8; j++) {
	if (wcrc & 0x0001) {
	    wcrc >>= 1;
	    wcrc ^= 0xa001;
	}
	else
	    wcrc >>= 1;
    }
}
return wcrc;
}

/**
 * @brief  Check return buffer by encoder.
 * @param  pData pointer to buffer
 * @param  address address of slave
 * @param  func encoder function code
 * @retval HAL status
 */
static HAL_StatusTypeDef Encoder_CheckRX(uint8_t *pData,uint8_t address,RS485_Enc_Func_t func){
    //Check return array contain right address and function code
    if(pData[0] != address || pData[1] != func)
	return HAL_ERROR;

    //Check CRC
    uint8_t total_byte = pData[2] + 3; //Addr+func+total_byte+[total_byte]
    uint16_t crc = Calculate_CRC(pData, total_byte);
    if(pData[total_byte + 1] != (uint8_t) ((crc >> 8) & 0xFF) ||
	    pData[total_byte + 2] != (uint8_t) ((crc >> 0) & 0xFF))
	return HAL_ERROR;

}


