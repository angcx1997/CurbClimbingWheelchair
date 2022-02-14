/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Button3_Pin GPIO_PIN_2
#define Button3_GPIO_Port GPIOE
#define LimitSW1_Pin GPIO_PIN_3
#define LimitSW1_GPIO_Port GPIOE
#define LimitSW2_Pin GPIO_PIN_4
#define LimitSW2_GPIO_Port GPIOE
#define LimitSW3_Pin GPIO_PIN_5
#define LimitSW3_GPIO_Port GPIOE
#define LimitSW4_Pin GPIO_PIN_6
#define LimitSW4_GPIO_Port GPIOE
#define AD_RANGE_Pin GPIO_PIN_0
#define AD_RANGE_GPIO_Port GPIOC
#define AD_OS2_Pin GPIO_PIN_1
#define AD_OS2_GPIO_Port GPIOC
#define AD_OS1_Pin GPIO_PIN_2
#define AD_OS1_GPIO_Port GPIOC
#define AD_OS0_Pin GPIO_PIN_3
#define AD_OS0_GPIO_Port GPIOC
#define AD_SPI1_CS_Pin GPIO_PIN_4
#define AD_SPI1_CS_GPIO_Port GPIOA
#define AD_SPI1_CLK_Pin GPIO_PIN_5
#define AD_SPI1_CLK_GPIO_Port GPIOA
#define AD_SPI1_MISO_Pin GPIO_PIN_6
#define AD_SPI1_MISO_GPIO_Port GPIOA
#define AD_BUSY_Pin GPIO_PIN_7
#define AD_BUSY_GPIO_Port GPIOA
#define AD_BUSY_EXTI_IRQn EXTI9_5_IRQn
#define AD_RST_Pin GPIO_PIN_4
#define AD_RST_GPIO_Port GPIOC
#define AD_CV_Pin GPIO_PIN_5
#define AD_CV_GPIO_Port GPIOC
#define ClimbM_IO_FR2_Pin GPIO_PIN_0
#define ClimbM_IO_FR2_GPIO_Port GPIOB
#define ClimbM_IO_EN2_Pin GPIO_PIN_1
#define ClimbM_IO_EN2_GPIO_Port GPIOB
#define ClimbM_IO_BRK2_Pin GPIO_PIN_7
#define ClimbM_IO_BRK2_GPIO_Port GPIOE
#define ClimbM_IO_ALM2_Pin GPIO_PIN_8
#define ClimbM_IO_ALM2_GPIO_Port GPIOE
#define Climb_TIM1_CH2_Pin GPIO_PIN_11
#define Climb_TIM1_CH2_GPIO_Port GPIOE
#define ClimbM_IO_FR1_Pin GPIO_PIN_12
#define ClimbM_IO_FR1_GPIO_Port GPIOE
#define ClimbM_IO_EN1_Pin GPIO_PIN_13
#define ClimbM_IO_EN1_GPIO_Port GPIOE
#define ClimbM_IO_BRK1_Pin GPIO_PIN_14
#define ClimbM_IO_BRK1_GPIO_Port GPIOE
#define ClimbM_IO_ALM1_Pin GPIO_PIN_15
#define ClimbM_IO_ALM1_GPIO_Port GPIOE
#define ClimbSpeed_TIM2_CH3_Pin GPIO_PIN_10
#define ClimbSpeed_TIM2_CH3_GPIO_Port GPIOB
#define HubM_UART3_TX_Pin GPIO_PIN_8
#define HubM_UART3_TX_GPIO_Port GPIOD
#define HubM_UART3_RX_Pin GPIO_PIN_9
#define HubM_UART3_RX_GPIO_Port GPIOD
#define CUI_SPI2_CS1_Pin GPIO_PIN_10
#define CUI_SPI2_CS1_GPIO_Port GPIOD
#define CUI_SPI2_CS2_Pin GPIO_PIN_11
#define CUI_SPI2_CS2_GPIO_Port GPIOD
#define HubM_IO_ALM_Pin GPIO_PIN_12
#define HubM_IO_ALM_GPIO_Port GPIOD
#define HubM_IO_SON_Pin GPIO_PIN_13
#define HubM_IO_SON_GPIO_Port GPIOD
#define HubM_IO_NOT_Pin GPIO_PIN_14
#define HubM_IO_NOT_GPIO_Port GPIOD
#define HubM_IO_POT_Pin GPIO_PIN_15
#define HubM_IO_POT_GPIO_Port GPIOD
#define Brake_Wheel_Pin GPIO_PIN_3
#define Brake_Wheel_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_7
#define Buzzer_GPIO_Port GPIOB
#define IMU_I2C1_SCL_Pin GPIO_PIN_8
#define IMU_I2C1_SCL_GPIO_Port GPIOB
#define IMU_I2C1_SDA_Pin GPIO_PIN_9
#define IMU_I2C1_SDA_GPIO_Port GPIOB
#define Button1_Pin GPIO_PIN_0
#define Button1_GPIO_Port GPIOE
#define Button2_Pin GPIO_PIN_1
#define Button2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
