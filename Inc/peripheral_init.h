/*
 * peripheral_init.h
 * - Configuration of all peripheral needed during initialization
 *  Created on: Feb 14, 2022
 *      Author: ray
 */

#ifndef PERIPHERAL_INIT_H_
#define PERIPHERAL_INIT_H_

#include "main.h"

extern CAN_HandleTypeDef hcan1;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

#endif /* PERIPHERAL_INIT_H_ */
