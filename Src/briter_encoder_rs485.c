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
static void Encoder_Send_Construct(Encoder_TX_t *txbuf, RS485_Enc_Func_t func, uint16_t own_addr, uint16_t send_addr, uint16_t read_byte);
static HAL_StatusTypeDef Encoder_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
static HAL_StatusTypeDef Encoder_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
static HAL_StatusTypeDef Encoder_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
static HAL_StatusTypeDef Encoder_CheckRX(uint8_t *pData, uint8_t address, RS485_Enc_Func_t func);
static uint16_t Calculate_CRC(uint8_t pbuf[], uint16_t num);
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

uint32_t BRITER_RS485_GetEncoderValue(Briter_Encoder_t *handler) {
    //Send encoder data
    Encoder_TX_t send_t;
    memset(&send_t, 0, sizeof(send_t));
    //2 as user want to read 2 different register to obtain encoder value
    Encoder_Send_Construct(&send_t, ENC_READ, handler->addr, BRITER_RS485_VALUE_ADDR, 2);
    if (Encoder_Transmit(handler->huart, send_t.buf, sizeof(send_t.buf)) != HAL_OK)
	return BRITER_RS485_ERROR;

    //Receive return from slave
    uint8_t receive_buf[9];
    if (Encoder_Receive(handler->huart, receive_buf, sizeof(receive_buf)) != HAL_OK)
	return BRITER_RS485_ERROR;

    //Check receive buffer
    if (Encoder_CheckRX(receive_buf, (uint8_t) (handler->addr), ENC_READ) != HAL_OK)
	return BRITER_RS485_ERROR;

    uint32_t encoder_value = receive_buf[3] << (3 * 8) | receive_buf[4] << (2 * 8) | receive_buf[5] << (1 * 8)
	    | receive_buf[6] << (0 * 8);
    return encoder_value;
}

HAL_StatusTypeDef BRITER_RS485_GetEncoderValue_DMA(Briter_Encoder_t *handler) {
    //Send encoder data
    Encoder_TX_t send_t;
    memset(&send_t, 0, sizeof(send_t));
    //2 as user want to read 2 different register to obtain encoder value
    Encoder_Send_Construct(&send_t, ENC_READ, handler->addr, BRITER_RS485_VALUE_ADDR, 2);
    return Encoder_Transmit_DMA(handler->huart, send_t.buf, sizeof(send_t.buf));
}

uint32_t BRITER_RS485_GetEncoderValue_DMA_Callback(Briter_Encoder_t *handler, uint8_t *pData) {
    //Check receive buffer
    if (Encoder_CheckRX(pData, (uint8_t) (handler->addr), ENC_READ) != HAL_OK)
	return BRITER_RS485_ERROR;
    uint32_t encoder_value = pData[3] << (3 * 8) | pData[4] << (2 * 8) | pData[5] << (1 * 8)
	    | pData[6] << (0 * 8);
    return encoder_value;
}

HAL_StatusTypeDef BRITER_RS485_SetBaudrate(Briter_Encoder_t *handler, RS485_Enc_Baudrate_t baudrate) {
    //Send encoder data
    Encoder_TX_t send_t;
    memset(&send_t, 0, sizeof(send_t));
    Encoder_Send_Construct(&send_t, ENC_WRITE_SINGLE, handler->addr, BRITER_RS485_BAUDRATE_ADDR, baudrate);
    if (Encoder_Transmit(handler->huart, send_t.buf, sizeof(send_t.buf)) != HAL_OK)
	return HAL_ERROR;
    //Receive return from slave
    uint8_t receive_buf[8];
    if (Encoder_Receive(handler->huart, receive_buf, sizeof(receive_buf)) != HAL_OK)
	return HAL_ERROR;
    //Check receive buffer
    if (Encoder_CheckRX(receive_buf, (uint8_t) (handler->addr), ENC_WRITE_SINGLE) != HAL_OK)
	return HAL_ERROR;
    //Check remaining byte other than CRC, addr, and func
    if (receive_buf[2] == send_t.buf[2] && receive_buf[3] == send_t.buf[3] && receive_buf[4] == send_t.buf[4]
	    && receive_buf[5] == send_t.buf[5])
	return HAL_OK;
    else
	return HAL_ERROR;
}

HAL_StatusTypeDef BRITER_RS485_SetDataMode(Briter_Encoder_t *handler, RS485_Enc_Mode_t mode) {
    //Send encoder data
    Encoder_TX_t send_t;
    memset(&send_t, 0, sizeof(send_t));
    Encoder_Send_Construct(&send_t, ENC_WRITE_SINGLE, handler->addr, BRITER_RS485_MODE_ADDR, mode);
    if (Encoder_Transmit(handler->huart, send_t.buf, sizeof(send_t.buf)) != HAL_OK)
	return HAL_ERROR;
    //Receive return from slave
    uint8_t receive_buf[8];
    if (Encoder_Receive(handler->huart, receive_buf, sizeof(receive_buf)) != HAL_OK)
	return HAL_ERROR;
    //Check receive buffer
    if (Encoder_CheckRX(receive_buf, (uint8_t) (handler->addr), ENC_WRITE_SINGLE) != HAL_OK)
	return HAL_ERROR;
    //Check remaining byte other than CRC, addr, and func
    if (receive_buf[2] == send_t.buf[2] && receive_buf[3] == send_t.buf[3] && receive_buf[4] == send_t.buf[4]
	    && receive_buf[5] == send_t.buf[5])
	return HAL_OK;
    else
	return HAL_ERROR;
}

HAL_StatusTypeDef BRITER_RS485_SetAddress(Briter_Encoder_t *handler, uint8_t to_address) {
    //Send encoder data
    Encoder_TX_t send_t;
    memset(&send_t, 0, sizeof(send_t));
    Encoder_Send_Construct(&send_t, ENC_WRITE_SINGLE, handler->addr, BRITER_RS485_ADDRESS_ADDR, to_address);
    if (Encoder_Transmit(handler->huart, send_t.buf, sizeof(send_t.buf)) != HAL_OK)
	return HAL_ERROR;
    //Receive return from slave
    uint8_t receive_buf[8];
    if (Encoder_Receive(handler->huart, receive_buf, sizeof(receive_buf)) != HAL_OK)
	return HAL_ERROR;
    //Check receive buffer
    if (Encoder_CheckRX(receive_buf, (uint8_t) (handler->addr), ENC_WRITE_SINGLE) != HAL_OK)
	return HAL_ERROR;
    //Check remaining byte other than CRC, addr, and func
    if (receive_buf[2] == send_t.buf[2] && receive_buf[3] == send_t.buf[3] && receive_buf[4] == send_t.buf[4]
	    && receive_buf[5] == send_t.buf[5])
	return HAL_OK;
    else
	return HAL_ERROR;
}

HAL_StatusTypeDef BRITER_RS485_SetReturnTime(Briter_Encoder_t *handler, uint16_t time) {
    //Send encoder data
    Encoder_TX_t send_t;
    memset(&send_t, 0, sizeof(send_t));
    Encoder_Send_Construct(&send_t, ENC_WRITE_SINGLE, handler->addr, BRITER_RS485_RETURN_TIME_ADDR, time);
    if (Encoder_Transmit(handler->huart, send_t.buf, sizeof(send_t.buf)) != HAL_OK)
	return HAL_ERROR;
    //Receive return from slave
    uint8_t receive_buf[8];
    if (Encoder_Receive(handler->huart, receive_buf, sizeof(receive_buf)) != HAL_OK)
	return HAL_ERROR;
    //Check receive buffer
    if (Encoder_CheckRX(receive_buf, (uint8_t) (handler->addr), ENC_WRITE_SINGLE) != HAL_OK)
	return HAL_ERROR;
    //Check remaining byte other than CRC, addr, and func
    if (receive_buf[2] == send_t.buf[2] && receive_buf[3] == send_t.buf[3] && receive_buf[4] == send_t.buf[4]
	    && receive_buf[5] == send_t.buf[5])
	return HAL_OK;
    else
	return HAL_ERROR;
}

HAL_StatusTypeDef BRITER_RS485_SetDirection(Briter_Encoder_t *handler, RS485_Enc_Direction_t direction) {
    //Send encoder data
    Encoder_TX_t send_t;
    memset(&send_t, 0, sizeof(send_t));
    Encoder_Send_Construct(&send_t, ENC_WRITE_SINGLE, handler->addr, BRITER_RS485_INCREASING_DIRECTION_ADDR, direction);
    if (Encoder_Transmit(handler->huart, send_t.buf, sizeof(send_t.buf)) != HAL_OK)
	return HAL_ERROR;
    //Receive return from slave
    uint8_t receive_buf[8];
    if (Encoder_Receive(handler->huart, receive_buf, sizeof(receive_buf)) != HAL_OK)
	return HAL_ERROR;
    //Check receive buffer
    if (Encoder_CheckRX(receive_buf, (uint8_t) (handler->addr), ENC_WRITE_SINGLE) != HAL_OK)
	return HAL_ERROR;
    //Check remaining byte other than CRC, addr, and func
    if (receive_buf[2] == send_t.buf[2] && receive_buf[3] == send_t.buf[3] && receive_buf[4] == send_t.buf[4]
	    && receive_buf[5] == send_t.buf[5])
	return HAL_OK;
    else
	return HAL_ERROR;
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
 * @brief  Use to construct message that need to be sent over to encoder.
 * @param  txbuf pointer to semdTx buffer
 * @param  own_addr encoder own address
 * @param  send_addr register of address that user want to access
 * @param  send_value depends on register user want to use
 * @retval none
 */
static void Encoder_Send_Construct(Encoder_TX_t *txbuf, RS485_Enc_Func_t func, uint16_t own_addr, uint16_t send_addr, uint16_t send_value) {
    txbuf->send_info.address = (uint8_t) own_addr;
    txbuf->send_info.function = func;
    txbuf->send_info.start_register[0] = (uint8_t) ((send_addr >> 8) & 0xFF);
    txbuf->send_info.start_register[1] = (uint8_t) ((send_addr >> 0) & 0xFF);
    uint16_t register_number = send_value;
    txbuf->send_info.register_number[0] = (uint8_t) ((register_number >> 8) & 0xFF);
    txbuf->send_info.register_number[1] = (uint8_t) ((register_number >> 0) & 0xFF);
    uint16_t crc = Calculate_CRC(txbuf->buf, sizeof(txbuf->buf) - 2);
    txbuf->send_info.crc[0] = (uint8_t) ((crc >> 0) & 0xFF);
    txbuf->send_info.crc[1] = (uint8_t) ((crc >> 8) & 0xFF);
}

/**
 * @brief  Check return buffer address, data func and crc by encoder.
 * @param  pData pointer to buffer
 * @param  address address of slave
 * @param  func encoder function code
 * @retval HAL status
 */
static HAL_StatusTypeDef Encoder_CheckRX(uint8_t *pData, uint8_t address, RS485_Enc_Func_t func) {
    //Check return array contain right address and function code
    if (pData[0] != address || pData[1] != func)
	return HAL_ERROR;

    //Check CRC
    uint8_t total_byte;
    if (pData[1] == ENC_READ) {
	//3 byte of READ return is size of byte follow
	//after total byte indicator and before CRC byte
	total_byte = pData[2] + 3; //Addr+func+total_byte+[total_byte]
    }
    else {
	total_byte = 8 - 2; //8 is total number of byte, 2 is byte for CRC
    }
    uint16_t crc = Calculate_CRC(pData, total_byte);
    if (pData[total_byte] != (uint8_t) ((crc >> 0) & 0xFF) || pData[total_byte + 1] != (uint8_t) ((crc >> 8) & 0xFF))
	return HAL_ERROR;

    return HAL_OK;
}

/**
 * @brief  Transmit encoder data via UART through polling mode.
 * @param  huart pointer to uart handler
 * @param  pData pointer to send buffer
 * @param  Size size of buffer
 * @retval HAL status
 */
static HAL_StatusTypeDef Encoder_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size) {
    return HAL_UART_Transmit(huart, pData, Size, 10);
}

/**
 * @brief  Transmit encoder data via UART through DMA mode.
 * @param  huart pointer to uart handler
 * @param  pData pointer to send buffer
 * @param  Size size of buffer
 * @retval HAL status
 */
static HAL_StatusTypeDef Encoder_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size) {
    return HAL_UART_Transmit_DMA(huart, pData, Size);
}

/**
 * @brief  Receive encoder data via UART through polling mode.
 * @param  huart pointer to uart handler
 * @param  pData pointer to send buffer
 * @param  Size size of buffer
 * @retval HAL status
 */
static HAL_StatusTypeDef Encoder_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size) {
    return HAL_UART_Receive(huart, pData, Size, 10);
}

