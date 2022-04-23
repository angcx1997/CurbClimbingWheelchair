/*
 * X2_6010S.h
 *	Motor Hub header file
 *  Created on: May 22, 2021
 *      Author: angcx
 */

#define HubM_UART3_TX_Pin 			GPIO_PIN_8
#define HubM_UART3_TX_GPIO_Port 	GPIOD
#define HubM_UART3_RX_Pin 			GPIO_PIN_9
#define HubM_UART3_RX_GPIO_Port 	GPIOD
#define HubM_IO_ALM_Pin 			GPIO_PIN_12
#define HubM_IO_ALM_GPIO_Port 		GPIOD
#define HubM_IO_SON_Pin 			GPIO_PIN_13
#define HubM_IO_SON_GPIO_Port 		GPIOD
#define HubM_IO_NOT_Pin 			GPIO_PIN_14
#define HubM_IO_NOT_GPIO_Port 		GPIOD
#define HubM_IO_POT_Pin 			GPIO_PIN_15
#define HubM_IO_POT_GPIO_Port 		GPIOD

#ifndef X2_6010S_H
#define X2_6010S_H

#include "stm32f4xx_hal.h"
#include <math.h>
#include "main.h"
#include <stdbool.h>

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

typedef struct{
  int32_t encoder_1;
  int32_t encoder_2;
}Encoder_Feedback;

extern Encoder_Feedback hub_encoder_feedback;

/**
  * @brief Initialize motor by setting all IO pin to LOW
  * @param None
  * @retval None
  */
void HubMotor_Init();

/**
  * @brief send speed command to Hub motor
  * Need add HAL_DMA_RECEIVE_CPT_CALLBACK to receive message
  * @param ang velocity
  * @retval HAL_StatusTypeDef hal status
  */
HAL_StatusTypeDef HubMotor_SendCommand(float m1_ang_speed, float m2_ang_speed);

/**
  * @brief Check receive buffer checksum
  * @param receive_buf receive buffer from uart
  * @retval true if receive buffer is valid
  */
bool HubMotor_CalculateChecksum(uint8_t receive_buf[]);

/**
  * @brief Used to receive data from callback function
  * @param receive_buf receive buffer from uart
  * @retval Encoder_Feedback
  */
Encoder_Feedback HubMotor_ReceiveCallback(uint8_t receive_buf[]);



#endif
