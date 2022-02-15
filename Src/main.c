/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "peripheral_init.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "adc.h"
#include "encoder.h"
#include "button.h"
#include "mpu6050.h"
#include "bd25l.h"
#include "X2_6010S.h"
//#include "wheelchair.h"
#include "PID.h"
#include "Sabertooth.h"
#include "joystick.h"
#include "differentialDrive.h"
#include "DifferentialDrivetoSabertooth.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
const float CLIMBING_LEG_LENGTH = 0.349; //in m, measured from the pivot to the center of hub motor
const float BASE_HEIGHT = 0.15;
const float BACK_BASE_HEIGHT = 0.15;

//ALLOWABLE is the maximum pos the climbing wheel can turn
//FRONT_CLIMBING is the pos that the base above the climbing wheel w
const uint32_t MAX_FRONT_ALLOWABLE_ENC = 3100;
const uint32_t MIN_FRONT_ALLOWABLE_ENC = 6600; //6600
const uint32_t MAX_FRONT_CLIMBING_ENC = 1950; //used for climbing up
const uint32_t MAX_BACK_ALLOWABLE_ENC = 3000;
const uint32_t MIN_BACK_ALLOWABLE_ENC = 7500;
const uint32_t MAX_BACK_CLIMBING_ENC = 1850; //used when climbing down
const uint32_t FRONT_FULL_ROTATION_ENC = 4096 * FRONT_GEAR_RATIO;
const uint32_t BACK_FULL_ROTATION_ENC = 4096 * BACK_GEAR_RATIO;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
	.name = "defaultTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* Task Handler */
TaskHandle_t task_control;
TaskHandle_t task_keyboard;
TaskHandle_t task_normalDrive;
TaskHandle_t task_climb_sensor;
TaskHandle_t task_joystick;
TaskHandle_t task_climbing;
TaskHandle_t task_usb;

QueueHandle_t queue_joystick;
QueueHandle_t encoder;

EncoderHandle encoderBack;
EncoderHandle encoderFront;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t motor_receive_buf[9];

//Hub Motor UART receive
uint8_t receive_buf[15];
Encoder_Feedback hub_encoder_feedback;

TickType_t last_can_rx_t[2] = {0}; //To keep track of CAN bus reception

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */
    BaseType_t status;
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    /* USER CODE BEGIN 2 */
    Peripheral_Init();
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    queue_joystick = xQueueCreate(5, sizeof(JoystickHandle)); //store pointer of joystick handler
    configASSERT(queue_joystick != NULL);
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    status = xTaskCreate(Task_Control, "Control Task", 250, NULL, 2, &task_control);
    configASSERT(status == pdPASS);
    status = xTaskCreate(Task_Keyboard, "Keyboard Task", 250, NULL, 2, &task_keyboard);
    configASSERT(status == pdPASS);
    status = xTaskCreate(Task_Climbing, "Climbing Task", 250, NULL, 2, &task_climbing);
    configASSERT(status == pdPASS);
    status = xTaskCreate(Task_Joystick, "Joystick Task", 250, NULL, 2, &task_joystick);
    configASSERT(status == pdPASS);
    status = xTaskCreate(Task_Climb_Sensor, "Climb Sensor Task", 300, NULL, 2, &task_climb_sensor);
    configASSERT(status == pdPASS);
    status = xTaskCreate(Task_NormalDrive, "Normal Drive Task", 250, NULL, 2, &task_normalDrive);
    configASSERT(status == pdPASS);
    status = xTaskCreate(Task_USB, "USB Task", 250, NULL, 2, &task_usb);
    configASSERT(status == pdPASS);

    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {
	    0
    };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {
	    0
    };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 72;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
	Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
	Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    switch (GPIO_Pin) {
	case AD_BUSY_Pin: {
	    int16_t adc_rawData[8];
	    JoystickHandle joystick_handler_irq;
	    ADC_Read(adc_rawData);
	    joystick_handler_irq.x = adc_rawData[2];
	    joystick_handler_irq.y = adc_rawData[1];
	    xQueueSendFromISR(queue_joystick, (void* )&joystick_handler_irq, &xHigherPriorityTaskWoken);
	    break;
	}
	default:
	    break;
    }
    /* Now the buffer is empty we can switch context if necessary. */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//Left Encoder Callback
    static CAN_RxHeaderTypeDef canRxHeader;
    uint8_t incoming[8];
    if (hcan == &hcan1) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canRxHeader, incoming);
	if (incoming[1] == ENC_ADDR_LEFT) {
	    ENCODER_Sort_Incoming(incoming, &encoderBack);
	    //Process the angle and GR
	    //4096 is encoder single turn value
	    //Need to check the encoder value in the correct direction
	    encoderBack.encoder_pos = (uint32_t) ((4096 * BACK_GEAR_RATIO) - encoderBack.encoder_pos)
		    % (4096 * BACK_GEAR_RATIO);
	    encoderBack.angleDeg = (float) encoderBack.encoder_pos / (4096 * BACK_GEAR_RATIO) * 360 + 36.587;
	    if (encoderBack.angleDeg > 360)
		encoderBack.angleDeg -= 360;
	    if (encoderBack.encoder_pos >= MAX_BACK_ALLOWABLE_ENC)
		encoderBack.signed_encoder_pos = encoderBack.encoder_pos - 4096 * BACK_GEAR_RATIO;

	    last_can_rx_t[BACK_INDEX] = xTaskGetTickCountFromISR();

	}
	if (incoming[1] == ENC_ADDR_RIGHT) {
	    ENCODER_Sort_Incoming(incoming, &encoderFront);
	    if (4096 * 24 - encoderFront.encoder_pos < 30000) {
		encoderFront.encoder_pos = (4096 * 24 - encoderFront.encoder_pos)
			% (uint32_t) (4096 * FRONT_GEAR_RATIO);
		encoderFront.angleDeg = (float) encoderFront.encoder_pos / (4096 * FRONT_GEAR_RATIO) * 360 + 36.587;
	    }
	    else {
		encoderFront.encoder_pos = (4096 * FRONT_GEAR_RATIO) - encoderFront.encoder_pos;
		encoderFront.angleDeg = (float) encoderFront.encoder_pos / (4096 * FRONT_GEAR_RATIO) * 360 + 36.587
			- 360;
	    }
	    if (encoderFront.encoder_pos >= MAX_FRONT_ALLOWABLE_ENC)
		encoderFront.signed_encoder_pos = encoderFront.encoder_pos - 4096 * FRONT_GEAR_RATIO;

	    last_can_rx_t[FRONT_INDEX] = xTaskGetTickCountFromISR();
	}
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    //Hub Encoder callback
    if (huart->Instance == USART3) {
	//Checksum, make sure that response is correct
	uint16_t sum = (uint16_t) receive_buf[0] + (uint16_t) receive_buf[1] + (uint16_t) receive_buf[2]
		+ (uint16_t) receive_buf[3] + (uint16_t) receive_buf[4] + (uint16_t) receive_buf[5]
		+ (uint16_t) receive_buf[6] + (uint16_t) receive_buf[7] + (uint16_t) receive_buf[8]
		+ (uint16_t) receive_buf[9] + (uint16_t) receive_buf[10] + (uint16_t) receive_buf[11]
		+ (uint16_t) receive_buf[12] + (uint16_t) receive_buf[13];
	if ((uint8_t) sum == receive_buf[14]) {
	    //Encoder Feedback
	    if (receive_buf[0] == 0xAA && receive_buf[1] == 0xA4 && receive_buf[3] == 0x00) {
		hub_encoder_feedback.encoder_1 = (receive_buf[9] << 24) + (receive_buf[8] << 16) + (receive_buf[7] << 8)
			+ (receive_buf[6]);
		hub_encoder_feedback.encoder_2 = (receive_buf[13] << 24) + (receive_buf[12] << 16)
			+ (receive_buf[11] << 8) + (receive_buf[10]);
	    }
	}
    }
    //Sabertooth Callback
//    if (huart->Instance == USART6) {
//	MotorProcessReply(&sabertooth_handler, motor_receive_buf, sizeof(motor_receive_buf));
//    }

    //curb change detection callback
//    if (huart->Instance == USART1) {
//	//how many bytes received
//	uint8_t len = RX_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
//	uint16_t distance = TFMINI_Plus_RcvData(pBuffer, len);
//	//send the received data;
//	if (distance != 0) {
//	    uint16_t distanceNoNoise = Noise_loop(distance);
//	    uint16_t diff = detectCurb_down(distanceNoNoise);
//	    if (diff >= CURBHEIGHT) {
//		//stop the base wheel completely
//		MotorThrottle(&sabertoothf_handler, 1, 0);
//		MotorThrottle(&sabertooth_handler, 2, 0);
//		lifting_mode = STOP;
//		//send the message to usb port
//		USB_TransmitData(CURB_CHANGE);
//	    }
//	    else {
//		USB_TransmitData(USB_MOVE);
//	    }
//	}
//	HAL_UART_Receive_DMA(&huart1, pBuffer, RX_SIZE);
//	last_uart_tx_time = HAL_GetTick();
//    }

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for (;;) {
	osDelay(1);
    }
    /* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6) {
	HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
