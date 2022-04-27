/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file         stm32f4xx_hal_msp.c
 * @brief        This file provides code for the MSP Initialization
 *               and de-Initialization codes.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_uart4_rx;

extern DMA_HandleTypeDef hdma_uart4_tx;

extern DMA_HandleTypeDef hdma_usart1_rx;

extern DMA_HandleTypeDef hdma_usart1_tx;

extern DMA_HandleTypeDef hdma_usart2_rx;

extern DMA_HandleTypeDef hdma_usart3_rx;

extern DMA_HandleTypeDef hdma_usart3_tx;

extern DMA_HandleTypeDef hdma_usart6_rx;

extern DMA_HandleTypeDef hdma_usart6_tx;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void) {
    /* USER CODE BEGIN MspInit 0 */

    /* USER CODE END MspInit 0 */

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* System interrupt init*/
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

    /* USER CODE BEGIN MspInit 1 */

    /* USER CODE END MspInit 1 */
}

/**
 * @brief CAN MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hcan: CAN handle pointer
 * @retval None
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan) {
    GPIO_InitTypeDef GPIO_InitStruct = {
	    0
    };
    if (hcan->Instance == CAN1) {
	/* USER CODE BEGIN CAN1_MspInit 0 */

	/* USER CODE END CAN1_MspInit 0 */
	/* Peripheral clock enable */
	__HAL_RCC_CAN1_CLK_ENABLE();

	__HAL_RCC_GPIOD_CLK_ENABLE();
	/**CAN1 GPIO Configuration
	 PD0     ------> CAN1_RX
	 PD1     ------> CAN1_TX
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* CAN1 interrupt Init */
	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	/* USER CODE BEGIN CAN1_MspInit 1 */

	/* USER CODE END CAN1_MspInit 1 */
    }

}

/**
 * @brief CAN MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hcan: CAN handle pointer
 * @retval None
 */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
	/* USER CODE BEGIN CAN1_MspDeInit 0 */

	/* USER CODE END CAN1_MspDeInit 0 */
	/* Peripheral clock disable */
	__HAL_RCC_CAN1_CLK_DISABLE();

	/**CAN1 GPIO Configuration
	 PD0     ------> CAN1_RX
	 PD1     ------> CAN1_TX
	 */
	HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);

	/* CAN1 interrupt DeInit */
	HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
	/* USER CODE BEGIN CAN1_MspDeInit 1 */

	/* USER CODE END CAN1_MspDeInit 1 */
    }

}

/**
 * @brief I2C MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct = {
	    0
    };
    if (hi2c->Instance == I2C1) {
	/* USER CODE BEGIN I2C1_MspInit 0 */

	/* USER CODE END I2C1_MspInit 0 */

	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**I2C1 GPIO Configuration
	 PB8     ------> I2C1_SCL
	 PB9     ------> I2C1_SDA
	 */
	GPIO_InitStruct.Pin = IMU_I2C1_SCL_Pin | IMU_I2C1_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Peripheral clock enable */
	__HAL_RCC_I2C1_CLK_ENABLE();
	/* USER CODE BEGIN I2C1_MspInit 1 */

	/* USER CODE END I2C1_MspInit 1 */
    }

}

/**
 * @brief I2C MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
	/* USER CODE BEGIN I2C1_MspDeInit 0 */

	/* USER CODE END I2C1_MspDeInit 0 */
	/* Peripheral clock disable */
	__HAL_RCC_I2C1_CLK_DISABLE();

	/**I2C1 GPIO Configuration
	 PB8     ------> I2C1_SCL
	 PB9     ------> I2C1_SDA
	 */
	HAL_GPIO_DeInit(IMU_I2C1_SCL_GPIO_Port, IMU_I2C1_SCL_Pin);

	HAL_GPIO_DeInit(IMU_I2C1_SDA_GPIO_Port, IMU_I2C1_SDA_Pin);

	/* USER CODE BEGIN I2C1_MspDeInit 1 */

	/* USER CODE END I2C1_MspDeInit 1 */
    }

}

/**
 * @brief SPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
    GPIO_InitTypeDef GPIO_InitStruct = {
	    0
    };
    if (hspi->Instance == SPI1) {
	/* USER CODE BEGIN SPI1_MspInit 0 */

	/* USER CODE END SPI1_MspInit 0 */
	/* Peripheral clock enable */
	__HAL_RCC_SPI1_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**SPI1 GPIO Configuration
	 PA5     ------> SPI1_SCK
	 PA6     ------> SPI1_MISO
	 */
	GPIO_InitStruct.Pin = AD_SPI1_CLK_Pin | AD_SPI1_MISO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN SPI1_MspInit 1 */

	/* USER CODE END SPI1_MspInit 1 */
    }

}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
	/* USER CODE BEGIN SPI1_MspDeInit 0 */

	/* USER CODE END SPI1_MspDeInit 0 */
	/* Peripheral clock disable */
	__HAL_RCC_SPI1_CLK_DISABLE();

	/**SPI1 GPIO Configuration
	 PA5     ------> SPI1_SCK
	 PA6     ------> SPI1_MISO
	 */
	HAL_GPIO_DeInit(GPIOA, AD_SPI1_CLK_Pin | AD_SPI1_MISO_Pin);

	/* USER CODE BEGIN SPI1_MspDeInit 1 */

	/* USER CODE END SPI1_MspDeInit 1 */
    }

}

/**
 * @brief TIM_Base MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base) {
    GPIO_InitTypeDef GPIO_InitStruct = {
	    0
    };
    if (htim_base->Instance == TIM1) {
	/* USER CODE BEGIN TIM1_MspInit 0 */

	/* USER CODE END TIM1_MspInit 0 */
	/* Peripheral clock enable */
	__HAL_RCC_TIM1_CLK_ENABLE();
	/* USER CODE BEGIN TIM1_MspInit 1 */

	/* USER CODE END TIM1_MspInit 1 */
    }
    else if (htim_base->Instance == TIM2) {
	/* USER CODE BEGIN TIM2_MspInit 0 */

	/* USER CODE END TIM2_MspInit 0 */
	/* Peripheral clock enable */
	__HAL_RCC_TIM2_CLK_ENABLE();
	/* USER CODE BEGIN TIM2_MspInit 1 */

	/* USER CODE END TIM2_MspInit 1 */
    }
    else if (htim_base->Instance == TIM3) {
	/* USER CODE BEGIN TIM3_MspInit 0 */

	/* USER CODE END TIM3_MspInit 0 */
	/* Peripheral clock enable */
	__HAL_RCC_TIM3_CLK_ENABLE();
	/* USER CODE BEGIN TIM3_MspInit 1 */

	/* USER CODE END TIM3_MspInit 1 */
    }
    else if (htim_base->Instance == TIM8) {
	/* USER CODE BEGIN TIM8_MspInit 0 */

	/* USER CODE END TIM8_MspInit 0 */
	/* Peripheral clock enable */
	__HAL_RCC_TIM8_CLK_ENABLE();

	__HAL_RCC_GPIOC_CLK_ENABLE();
	/**TIM8 GPIO Configuration
	 PC9     ------> TIM8_CH4
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* TIM8 interrupt Init */
	HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
	HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
	HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
	HAL_NVIC_SetPriority(TIM8_CC_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
	/* USER CODE BEGIN TIM8_MspInit 1 */

	/* USER CODE END TIM8_MspInit 1 */
    }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim) {
    GPIO_InitTypeDef GPIO_InitStruct = {
	    0
    };
    if (htim->Instance == TIM1) {
	/* USER CODE BEGIN TIM1_MspPostInit 0 */

	/* USER CODE END TIM1_MspPostInit 0 */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	/**TIM1 GPIO Configuration
	 PE11     ------> TIM1_CH2
	 */
	GPIO_InitStruct.Pin = Climb_TIM1_CH2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(Climb_TIM1_CH2_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN TIM1_MspPostInit 1 */

	/* USER CODE END TIM1_MspPostInit 1 */
    }
    else if (htim->Instance == TIM2) {
	/* USER CODE BEGIN TIM2_MspPostInit 0 */

	/* USER CODE END TIM2_MspPostInit 0 */

	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**TIM2 GPIO Configuration
	 PB10     ------> TIM2_CH3
	 */
	GPIO_InitStruct.Pin = ClimbSpeed_TIM2_CH3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(ClimbSpeed_TIM2_CH3_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN TIM2_MspPostInit 1 */

	/* USER CODE END TIM2_MspPostInit 1 */
    }

}
/**
 * @brief TIM_Base MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base) {
    if (htim_base->Instance == TIM1) {
	/* USER CODE BEGIN TIM1_MspDeInit 0 */

	/* USER CODE END TIM1_MspDeInit 0 */
	/* Peripheral clock disable */
	__HAL_RCC_TIM1_CLK_DISABLE();
	/* USER CODE BEGIN TIM1_MspDeInit 1 */

	/* USER CODE END TIM1_MspDeInit 1 */
    }
    else if (htim_base->Instance == TIM2) {
	/* USER CODE BEGIN TIM2_MspDeInit 0 */

	/* USER CODE END TIM2_MspDeInit 0 */
	/* Peripheral clock disable */
	__HAL_RCC_TIM2_CLK_DISABLE();
	/* USER CODE BEGIN TIM2_MspDeInit 1 */

	/* USER CODE END TIM2_MspDeInit 1 */
    }
    else if (htim_base->Instance == TIM3) {
	/* USER CODE BEGIN TIM3_MspDeInit 0 */

	/* USER CODE END TIM3_MspDeInit 0 */
	/* Peripheral clock disable */
	__HAL_RCC_TIM3_CLK_DISABLE();
	/* USER CODE BEGIN TIM3_MspDeInit 1 */

	/* USER CODE END TIM3_MspDeInit 1 */
    }
    else if (htim_base->Instance == TIM8) {
	/* USER CODE BEGIN TIM8_MspDeInit 0 */

	/* USER CODE END TIM8_MspDeInit 0 */
	/* Peripheral clock disable */
	__HAL_RCC_TIM8_CLK_DISABLE();

	/**TIM8 GPIO Configuration
	 PC9     ------> TIM8_CH4
	 */
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);

	/* TIM8 interrupt DeInit */
	HAL_NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
	HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
	HAL_NVIC_DisableIRQ(TIM8_TRG_COM_TIM14_IRQn);
	HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);
	/* USER CODE BEGIN TIM8_MspDeInit 1 */

	/* USER CODE END TIM8_MspDeInit 1 */
    }

}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    GPIO_InitTypeDef GPIO_InitStruct = {
	    0
    };
    if (huart->Instance == UART4) {
	/* USER CODE BEGIN UART4_MspInit 0 */

	/* USER CODE END UART4_MspInit 0 */
	/* Peripheral clock enable */
	__HAL_RCC_UART4_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**UART4 GPIO Configuration
	 PA0/WKUP     ------> UART4_TX
	 PA1     ------> UART4_RX
	 */
	GPIO_InitStruct.Pin = Base_Encoder_TX_Pin | Base_Encoder_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* UART4 DMA Init */
	/* UART4_RX Init */
	hdma_uart4_rx.Instance = DMA1_Stream2;
	hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_uart4_rx.Init.Mode = DMA_NORMAL;
	hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK) {
	    Error_Handler();
	}

	__HAL_LINKDMA(huart, hdmarx, hdma_uart4_rx);
	/* UART4_TX Init */
	hdma_uart4_tx.Instance = DMA1_Stream4;
	hdma_uart4_tx.Init.Channel = DMA_CHANNEL_4;
	hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_uart4_tx.Init.Mode = DMA_NORMAL;
	hdma_uart4_tx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_uart4_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_uart4_tx) != HAL_OK)
	{
	  Error_Handler();
	}

	__HAL_LINKDMA(huart,hdmatx,hdma_uart4_tx);
	/* USART1 interrupt Init */
	HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(UART4_IRQn);
	/* USER CODE BEGIN UART4_MspInit 1 */

	/* USER CODE END UART4_MspInit 1 */
    }

    else if (huart->Instance == USART1) {
	/* USER CODE BEGIN USART1_MspInit 0 */

	/* USER CODE END USART1_MspInit 0 */
	/* Peripheral clock enable */
	__HAL_RCC_USART1_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USART1 GPIO Configuration
	 PA9     ------> USART1_TX
	 PA10     ------> USART1_RX
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USART1 DMA Init */
	/* USART1_RX Init */
	hdma_usart1_rx.Instance = DMA2_Stream2;
	hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart1_rx.Init.Mode = DMA_NORMAL;
	hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) {
	    Error_Handler();
	}

	__HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);

	/* USART1_TX Init */
	hdma_usart1_tx.Instance = DMA2_Stream7;
	hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart1_tx.Init.Mode = DMA_NORMAL;
	hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) {
	    Error_Handler();
	}

	__HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

	/* USART1 interrupt Init */
	HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	/* USER CODE BEGIN USART1_MspInit 1 */

	/* USER CODE END USART1_MspInit 1 */
    }
    else if (huart->Instance == USART2) {
	/* USER CODE BEGIN USART2_MspInit 0 */

	/* USER CODE END USART2_MspInit 0 */
	/* Peripheral clock enable */
	__HAL_RCC_USART2_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USART2 GPIO Configuration
	 PA2     ------> USART2_TX
	 PA3     ------> USART2_RX
	 */
	GPIO_InitStruct.Pin = Battery_TX_Pin | Battery_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USART2 DMA Init */
	/* USART2_RX Init */
	hdma_usart2_rx.Instance = DMA1_Stream5;
	hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_rx.Init.Mode = DMA_NORMAL;
	hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
	{
	  Error_Handler();
	}

	__HAL_LINKDMA(huart,hdmarx,hdma_usart2_rx);

	/* USART2 interrupt Init */
	HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	/* USER CODE BEGIN USART2_MspInit 1 */

	/* USER CODE END USART2_MspInit 1 */
    }
    else if (huart->Instance == USART3) {
	/* USER CODE BEGIN USART3_MspInit 0 */

	/* USER CODE END USART3_MspInit 0 */
	/* Peripheral clock enable */
	__HAL_RCC_USART3_CLK_ENABLE();

	__HAL_RCC_GPIOD_CLK_ENABLE();
	/**USART3 GPIO Configuration
	 PD8     ------> USART3_TX
	 PD9     ------> USART3_RX
	 */
	GPIO_InitStruct.Pin = HubM_UART3_TX_Pin | HubM_UART3_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* USART3 DMA Init */
	/* USART3_RX Init */
	hdma_usart3_rx.Instance = DMA1_Stream1;
	hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart3_rx.Init.Mode = DMA_NORMAL;
	hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) {
	    Error_Handler();
	}

	__HAL_LINKDMA(huart, hdmarx, hdma_usart3_rx);

	/* USART3_TX Init */
	hdma_usart3_tx.Instance = DMA1_Stream3;
	hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart3_tx.Init.Mode = DMA_NORMAL;
	hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) {
	    Error_Handler();
	}

	__HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

	/* USART3 interrupt Init */
	HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	/* USER CODE BEGIN USART3_MspInit 1 */

	/* USER CODE END USART3_MspInit 1 */
    }
    else if (huart->Instance == USART6) {
	/* USER CODE BEGIN USART6_MspInit 0 */

	/* USER CODE END USART6_MspInit 0 */
	/* Peripheral clock enable */
	__HAL_RCC_USART6_CLK_ENABLE();

	__HAL_RCC_GPIOC_CLK_ENABLE();
	/**USART6 GPIO Configuration
	 PC6     ------> USART6_TX
	 PC7     ------> USART6_RX
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* USART6 DMA Init */
	/* USART6_RX Init */
	hdma_usart6_rx.Instance = DMA2_Stream1;
	hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
	hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart6_rx.Init.Mode = DMA_NORMAL;
	hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK) {
	    Error_Handler();
	}

	__HAL_LINKDMA(huart, hdmarx, hdma_usart6_rx);

	/* USART6_TX Init */
	hdma_usart6_tx.Instance = DMA2_Stream6;
	hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
	hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart6_tx.Init.Mode = DMA_NORMAL;
	hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK) {
	    Error_Handler();
	}

	__HAL_LINKDMA(huart, hdmatx, hdma_usart6_tx);

	/* USART6 interrupt Init */
	HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART6_IRQn);
	/* USER CODE BEGIN USART6_MspInit 1 */

	/* USER CODE END USART6_MspInit 1 */
    }

}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
	/* USER CODE BEGIN USART1_MspDeInit 0 */

	/* USER CODE END USART1_MspDeInit 0 */
	/* Peripheral clock disable */
	__HAL_RCC_USART1_CLK_DISABLE();

	/**USART1 GPIO Configuration
	 PA9     ------> USART1_TX
	 PA10     ------> USART1_RX
	 */
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

	/* USART1 DMA DeInit */
	HAL_DMA_DeInit(huart->hdmarx);
	HAL_DMA_DeInit(huart->hdmatx);

	/* USART1 interrupt DeInit */
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	/* USER CODE BEGIN USART1_MspDeInit 1 */

	/* USER CODE END USART1_MspDeInit 1 */
    }
    else if (huart->Instance == USART3) {
	/* USER CODE BEGIN USART3_MspDeInit 0 */

	/* USER CODE END USART3_MspDeInit 0 */
	/* Peripheral clock disable */
	__HAL_RCC_USART3_CLK_DISABLE();

	/**USART3 GPIO Configuration
	 PD8     ------> USART3_TX
	 PD9     ------> USART3_RX
	 */
	HAL_GPIO_DeInit(GPIOD, HubM_UART3_TX_Pin | HubM_UART3_RX_Pin);

	/* USART3 DMA DeInit */
	HAL_DMA_DeInit(huart->hdmarx);
	HAL_DMA_DeInit(huart->hdmatx);

	/* USART3 interrupt DeInit */
	HAL_NVIC_DisableIRQ(USART3_IRQn);
	/* USER CODE BEGIN USART3_MspDeInit 1 */

	/* USER CODE END USART3_MspDeInit 1 */
    }
    else if (huart->Instance == USART6) {
	/* USER CODE BEGIN USART6_MspDeInit 0 */

	/* USER CODE END USART6_MspDeInit 0 */
	/* Peripheral clock disable */
	__HAL_RCC_USART6_CLK_DISABLE();

	/**USART6 GPIO Configuration
	 PC6     ------> USART6_TX
	 PC7     ------> USART6_RX
	 */
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7);

	/* USART6 DMA DeInit */
	HAL_DMA_DeInit(huart->hdmarx);
	HAL_DMA_DeInit(huart->hdmatx);

	/* USART6 interrupt DeInit */
	HAL_NVIC_DisableIRQ(USART6_IRQn);
	/* USER CODE BEGIN USART6_MspDeInit 1 */

	/* USER CODE END USART6_MspDeInit 1 */
    }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
