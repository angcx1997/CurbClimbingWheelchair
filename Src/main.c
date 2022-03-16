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
#include "tfmini.h"

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
const uint32_t MAX_BACK_CLIMBING_ENC = 1950; //used when climbing down
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

QueueHandle_t queue_joystick_raw;
QueueHandle_t encoder;

TimerHandle_t timer_buzzer;
uint8_t buzzer_expiry_count = 0; //Zero means no expiry count

EncoderHandle encoderBack;
EncoderHandle encoderFront;

//Buffer to receive uart tf-mini data
uint8_t tf_rx_buf[TFMINI_RX_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void Buzzer_Timer_Callback(TimerHandle_t xTimer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t motor_receive_buf[9];

//Hub Motor UART receive
uint8_t receive_buf[15];
Encoder_Feedback hub_encoder_feedback;

TickType_t last_can_rx_t[2] = {
	0
}; //To keep track of CAN bus reception
TickType_t last_hub_rx_t = 0; //Keep track of HUB reception activity
TickType_t last_tf_mini_t = 0;

//Front Climbing Position Control
struct pid_controller frontClimb_ctrl;
PID_t frontClimb_pid;
float frontClimb_input = 0, frontClimb_output = 0;
float frontClimb_setpoint = 0;
float frontClimb_kp = 0.35, frontClimb_ki = 0.003, frontClimb_kd = 0.00001;

//Back Climbing Position Control
struct pid_controller backClimb_ctrl;
PID_t backClimb_pid;
float backClimb_input = 0, backClimb_output = 0;
float backClimb_setpoint = 0;
float backClimb_kp = 0.3, backClimb_ki = 0.004, backClimb_kd = 0.00001;

//TF-mini
uint16_t distance = 0;
uint16_t distanceNoNoise;
int diff;
uint8_t usbBuffer[1];

static uint16_t prev_dist = 0;
extern Operation_Mode lifting_mode;
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

    //Initialize front and back climbing position controller
    frontClimb_pid = pid_create(&frontClimb_ctrl, &frontClimb_input, &frontClimb_output, &frontClimb_setpoint,
	    frontClimb_kp, frontClimb_ki, frontClimb_kd);
    pid_limits(frontClimb_pid, -80, 80);
    pid_sample(frontClimb_pid, 1);
    pid_auto(frontClimb_pid);

    backClimb_pid = pid_create(&backClimb_ctrl, &backClimb_input, &backClimb_output, &backClimb_setpoint, backClimb_kp,
	    backClimb_ki, backClimb_kd);
    pid_limits(backClimb_pid, -80, 80);
    pid_sample(backClimb_pid, 1);
    pid_auto(backClimb_pid);

//    HAL_UART_Receive_DMA(&huart3, receive_buf, 15);
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
    timer_buzzer = xTimerCreate("Buzzer Timer", pdMS_TO_TICKS(500), pdTRUE, NULL, Buzzer_Timer_Callback);
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    queue_joystick_raw = xQueueCreate(5, sizeof(JoystickHandle)); //store joystick handler
    configASSERT(queue_joystick_raw != NULL);
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
	/* Callback from joystick reading*/
	case AD_BUSY_Pin: {
	    int16_t adc_rawData[8];
	    JoystickHandle joystick_handler_irq;
	    ADC_Read(adc_rawData);
	    joystick_handler_irq.x = adc_rawData[2];
	    joystick_handler_irq.y = adc_rawData[1];
	    xQueueSendFromISR(queue_joystick_raw, (void* )&joystick_handler_irq, &xHigherPriorityTaskWoken);
	    break;
	}
	default:
	    break;
    }
    /* Now the buffer is empty we can switch context if necessary. */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    static CAN_RxHeaderTypeDef canRxHeader;
    uint8_t incoming[8];
    if (hcan == &hcan1) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canRxHeader, incoming);
	/**Process the angle with gear ratio
	 * 4096 is encoder single turn value
	 * 24 is the encoder maximum multi-turn value
	 * Need to make sure and check the encoder value in the correct direction
	 */
	//Left encoder callback
	if (incoming[1] == ENC_ADDR_LEFT) {
	    ENCODER_Sort_Incoming(incoming, &encoderBack);

	    encoderBack.encoder_pos = (uint32_t) ((4096 * BACK_GEAR_RATIO) - encoderBack.encoder_pos)
		    % (4096 * BACK_GEAR_RATIO);
	    encoderBack.angleDeg = (float) encoderBack.encoder_pos / (4096 * BACK_GEAR_RATIO) * 360 + 36.587;
	    if (encoderBack.angleDeg > 360)
		encoderBack.angleDeg -= 360;
	    if (encoderBack.encoder_pos >= MAX_BACK_ALLOWABLE_ENC)
		encoderBack.signed_encoder_pos = encoderBack.encoder_pos - 4096 * BACK_GEAR_RATIO;

	    last_can_rx_t[BACK_INDEX] = xTaskGetTickCountFromISR();

	}
	//Right encoder callback
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
	    if (receive_buf[0] == 0xAA && receive_buf[1] == 0xA4 && receive_buf[3] == 0x00 && receive_buf[4] == 0x00) {
		hub_encoder_feedback.encoder_1 = (receive_buf[9] << 24) + (receive_buf[8] << 16) + (receive_buf[7] << 8)
			+ (receive_buf[6]);
		hub_encoder_feedback.encoder_2 = (receive_buf[13] << 24) + (receive_buf[12] << 16)
			+ (receive_buf[11] << 8) + (receive_buf[10]);
		last_hub_rx_t = xTaskGetTickCountFromISR();
	    }
	}
    }
#if 0
    //Sabertooth Callback
    //TODO: Didnt use as not enough pin for data reception
    //Note that DO NOT use the same uart for motor speed transmission and data transmission
    //Will cause latency when transmitting speed and really dangerous
    if (huart->Instance == USART6) {
	MotorProcessReply(&sabertooth_handler, motor_receive_buf, sizeof(motor_receive_buf));
    }
#endif

    //curb change detection callback
    if (huart->Instance == USART1) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//how many bytes received

	uint8_t len = TFMINI_RX_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
	distance = TFMINI_Plus_RcvData(tf_rx_buf, len);
	//send the received data;
	if (distance != 0) {
	    distanceNoNoise = Noise_loop(distance);
	    // diff = detectCurb_down(distanceNoNoise);
	    if (prev_dist == 0)
		prev_dist = distanceNoNoise;
	    /*
	     diff = distanceNoNoise - prev_dist;
	     if(diff >= 15 && lifting_mode == NORMAL){
	     //STOP the base wheel
	     lifting_mode = STOP;
	     xTaskNotifyFromISR(task_normalDrive, 0, eNoAction, &xHigherPriorityTaskWoken);
	     USB_TransmitData(CURB_CHANGE);
	     }
	     else if (diff >= 15 && lifting_mode == CURB_DETECTED){
	     //stop the base wheel
	     lifting_mode = NORMAL;
	     xTaskNotifyFromISR(task_normalDrive, 0, eNoAction, &xHigherPriorityTaskWoken);
	     USB_TransmitData(USB_MOVE);
	     }
	     else if(lifting_mode == NORMAL){
	     USB_TransmitData(USB_MOVE);
	     }
	     */
	    if (distanceNoNoise >= 20 && lifting_mode == NORMAL) {
		//stop the base wheel completely
		lifting_mode = STOP;
		xTaskNotifyFromISR(task_normalDrive, 0, eNoAction, &xHigherPriorityTaskWoken);
		//send the message to usb port
		USB_TransmitData(CURB_CHANGE);
	    }
	    else if (distanceNoNoise < 20 && lifting_mode == CURB_DETECTED) {
		//start the base wheel
		lifting_mode = NORMAL;
		xTaskNotifyFromISR(task_normalDrive, 0, eNoAction, &xHigherPriorityTaskWoken);
		USB_TransmitData(USB_MOVE);
	    }
	    else if (lifting_mode == NORMAL) {
		USB_TransmitData(USB_MOVE);
	    }

	    prev_dist = distanceNoNoise;
//			prev_dist = MAX(prev_dist, distanceNoNoise);
//			if (lifting_mode == STOP)
//				prev_dist = 0;
	}

	last_tf_mini_t = xTaskGetTickCountFromISR();
	HAL_UART_Receive_DMA(&huart1, tf_rx_buf, TFMINI_RX_SIZE);
	/* Now the buffer is empty we can switch context if necessary. */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

}

/**
 * @brief  Software timer to Toggle buzzer periodically.
 * @param  xTimer: freertos software timer
 * @retval None
 */
void Buzzer_Timer_Callback(TimerHandle_t xTimer) {
    /* Do not use a block time if calling a timer API function
     from a timer callback function, as doing so could cause a
     deadlock!
     */
    HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);

#if 0
    //TODO: If want timer to beep at "expiry_count
    //Havent tested
    static uint8_t count = 0;
    count = (buzzer_expiry_count == 0) ? 0 : count++;
    if (count >= buzzer_expiry_count * 2)
	xTimerStop(xTimer, 0);
#endif
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
