/**
  ******************************************************************************
  * @file    briter_encoder_rs485.h
  * @author  Ang Chin Xian
  * @brief   Briter encoder RS485 Driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the encoder via RS485:
  *           + Initialization functions
  *           + Configuration functions
  *           + Read and get function
  *
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  1. Create handler to hold Briter_Encoder_t and input the correct address and
      UART handler for the specific driver
  2. Make sure baudrate is match,  data length 8 bit, 0 parity, 1 stop bit
  3. Default encoder address is 1 and baudrate is 9600bps if no configure
  4. All configuration function is performed through polling mode
  5. For reading encoder value,
      - In this driver, user is not interested in getting single turn encoder value
      - Encoder value is depends on the hardware itself
      - Polling Mode
	  BRITER_RS485_GetEncoderValue()
      - DMA Mode
	  a. Call BRITER_RS485_GetEncoderValue_DMA() in main
	  b. Add HAL_UART_DMA_TxCplt_Callback() to code
	  c. In TxCmpltCallback, add
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) RxBuf, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	  d. Add HAL_UARTEx_RxEventCallback()
	      Call BRITER_RS485_GetEncoderValue_RX_Callback()


*/
#ifndef BRITER_ENCODER_RS485_H_
#define BRITER_ENCODER_RS485_H_

#include <stdint.h>
#include <stm32f4xx.h>

typedef struct {
    uint8_t addr;
    uint32_t encoder_value;
    UART_HandleTypeDef *huart;
} Briter_Encoder_t;

/** @defgroup BRITER_ENCODER_RS485_Exported_Constants
 * @{
 */
/******************************************************************************/
/*************************** Encoder REGISTER MAPPING  **************************/
/******************************************************************************/
#define BRITER_RS485_VALUE_ADDR			0x00	/*!< Read encoder value*/
#define BRITER_RS485_NO_OF_TURN_ADDR		0x02	/*!< Read encoder number of turns*/
#define BRITER_RS485_SINGLE_TURN_ADDR		0x03	/*!< Read encoder single turn value*/
#define BRITER_RS485_ADDRESS_ADDR		0x04	/*!< Set encoder communication address*/
#define BRITER_RS485_BAUDRATE_ADDR		0x05	/*!< Set encoder baudrate*/
#define BRITER_RS485_MODE_ADDR			0x06	/*!< Set encoder mode*/
#define BRITER_RS485_RETURN_TIME_ADDR		0x07	/*!< Set automatic return time*/
#define BRITER_RS485_RESET_ZERO_ADDR		0x08	/*!< set encoder reset zero mark*/
#define BRITER_RS485_INCREASING_DIRECTION_ADDR	0x09	/*!< Set encoder value increasing direciton*/
#define BRITER_RS485_SET_POSITION_ADDR		0x0B	/*!< Set current value of encoder (use WRITE_MULTI)*/
#define BRITER_RS485_SET_MIDPOINT_ADDR		0x0E	/*!< Set encoder midpoint*/
#define BRITER_RS485_SET_MUL_5_ADDR		0x0F	/*!< Set current turn value to 5 turns*/

/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/

/** @defgroup Briter RS485 Baudrate Selection
 * @{
 */
typedef enum {
    RS485_ENC_BAUDRATE_9600 = 0x00,
    RS485_ENC_BAUDRATE_19200,
    RS485_ENC_BAUDRATE_38400,
    RS485_ENC_BAUDRATE_57600,
    RS485_ENC_BAUDRATE_115200,
} RS485_Enc_Baudrate_t;
/**
 * @}
 */

/** @defgroup Briter RS485 Mode Selection
 * @{
 */
typedef enum {
    RS485_ENC_MODE_QUERY = 0x00,
    RS485_ENC_MODE_BACKHAUL,
} RS485_Enc_Mode_t;
/**
 * @}
 */

/** @defgroup Briter RS485 Incresing Direction Selection
 * @{
 */
typedef enum {
    RS485_ENC_DIRECTION_CLOCKWISE = 0x00,
    RS485_ENC_DIRECTION_COUNTERCLOCKWISE,
} RS485_Enc_Direction_t;
/**
 * @}
 */

/** @defgroup Briter RS485 Error Message
 * @{
 */
#define BRITER_RS485_ERROR	-1
/**
 * @}
 */

/** @defgroup Briter_RS485_Exported_Functions
 * @{
 */
/**
* @brief  Initialize encoder handler.
* @param  handler: encoder handler
* @param  address: encoder address
* @param  huart: uart handler
* @retval HAL status
*/
HAL_StatusTypeDef BRITER_RS485_Init(Briter_Encoder_t* handler,uint8_t address, UART_HandleTypeDef* huart);

/**
* @brief  Get encoder value though POLLING MODE.
* @param  handler: encoder handler to give address and store encoder return value
* @retval encoder value, -1 if error occurs
*/
uint32_t BRITER_RS485_GetEncoderValue(Briter_Encoder_t* handler);

/**
* @brief  Send info to encoder to read through DMA.
* @param  handler: encoder handler
* @retval HAL status
* @note   If complete, DMA_TX_Cplt will be called if DMA interrupt is activated
*/
HAL_StatusTypeDef BRITER_RS485_GetEncoderValue_DMA(Briter_Encoder_t* handler);

/**
* @brief  Get encoder value though DMA during reception.
* @param  handler: encoder handler
* @retval HAL status
* @note   Use inside DMA RX Idle Callback
*/
uint32_t BRITER_RS485_GetEncoderValue_DMA_Callback(Briter_Encoder_t* handler, uint8_t *pData);

/**
* @brief Set encoder baudrate.
* @param  handler: encoder handler
* @param  baudrate: refer to @Briter RS485 Baudrate Selection
* @retval HAL status
*/
HAL_StatusTypeDef BRITER_RS485_SetBaudrate(Briter_Encoder_t* handler, RS485_Enc_Baudrate_t baudrate);

/**
* @brief Change encoder address.
* @param  handler: encoder handler
* @param  to_address: the address you want to change to
* @retval HAL status
*/
HAL_StatusTypeDef BRITER_RS485_SetAddress(Briter_Encoder_t* handler, uint8_t to_address);

/**
* @brief Set data mode to query or backhaul
* @param  handler: encoder handler
* @param  mode : refer to @Briter RS485 Mode Selection
* @retval HAL status
*/
HAL_StatusTypeDef BRITER_RS485_SetDataMode(Briter_Encoder_t* handler, RS485_Enc_Mode_t mode);

/**
* @brief Set encoder return value
* @param  handler: encoder handler
* @param  time : range from 0-65535 (ms)
* @retval HAL status
* @note Default 50ms.  once the automatic return time is set to less than
* 	20 milliseconds, the encoder will not be
* 	able to set other parameters, use with caution!
*/
HAL_StatusTypeDef BRITER_RS485_SetReturnTime(Briter_Encoder_t* handler, uint16_t time);

/**
* @brief Set encoder return value
* @param  handler: encoder handler
* @param  direction : refer to @Briter RS485 Incresing Direction Selection
* @retval HAL status
*/
HAL_StatusTypeDef BRITER_RS485_SetDirection(Briter_Encoder_t* handler, RS485_Enc_Direction_t direction);

/**
 * @}
 */
#endif /* BRITER_ENCODER_RS485_H_ */
