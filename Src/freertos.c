/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "queue.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define TO_RAD(x) x * M_PI / 180
#define TO_DEG(x) x * 180 / M_PI

#define BITCHECK(byte,nbit) ((byte) & 1<<(nbit))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern TaskHandle_t task_keyboard;
extern TaskHandle_t task_normalDrive;
extern TaskHandle_t task_encoder;
extern TaskHandle_t task_joystick;
extern TaskHandle_t task_climbing;
extern TaskHandle_t task_usb;

extern QueueHandle_t queue_joystick;
extern QueueHandle_t encoder;

extern EncoderHandle encoderBack;
extern EncoderHandle encoderFront;

int speed[2] = {
	0
};
Motor_TypeDef rearMotor, backMotor; //declare in bd25l.c
uint8_t state = 0;

int x, y;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Task_Keyboard(void *param) {
    //Memset struct to 0 and Initialize all button used
    Button_TypeDef button1, button2, button3;
    memset(&button1, 0, sizeof(Button_TypeDef));
    memset(&button2, 0, sizeof(Button_TypeDef));
    memset(&button3, 0, sizeof(Button_TypeDef));

    button1.gpioPort = Button1_GPIO_Port;
    button1.gpioPin = Button1_Pin;
    button2.gpioPort = Button2_GPIO_Port;
    button2.gpioPin = Button2_Pin;
    button3.gpioPort = Button3_GPIO_Port;
    button3.gpioPin = Button3_Pin;

    TickType_t tick = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10); //execution period

    while (1) {
	GPIO_Digital_Filtered_Input(&button1, 30);
	GPIO_Digital_Filtered_Input(&button2, 30);
	GPIO_Digital_Filtered_Input(&button3, 30);

	if (button1.state == 1)
	    state |= 1 << 0;
	else
	    state &= ~(1 << 0);

	if (button2.state == 1)
	    state |= 1 << 1;
	else
	    state &= ~(1 << 1);

	if (button3.state == 1)
	    state |= 1 << 2;
	else
	    state &= ~(1 << 2);

	vTaskDelayUntil(&tick, period);
    }
}

void Task_NormalDrive(void *param) {
    while (1) {
	vTaskDelay(1000);
    }
}

void Task_Encoder(void *param) {
    ENCODER_Init();
    while (1) {
	ENCODER_Get_Angle(&encoderBack);
	vTaskDelay(50);
	ENCODER_Get_Angle(&encoderFront);
	vTaskDelay(50);
    }
}

void Task_Joystick(void *param) {

    TickType_t tick = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100); //execution period

    JoystickHandle joystick_handler;

    //Initialize AD7606
    ADC_Init();

    while (1) {
	ADC_DataRequest();
	if (xQueueReceive(queue_joystick, &(joystick_handler), 50) == pdPASS) {
	    calculatePos(&joystick_handler);
	}
	else {
	    joystick_handler.x = 0;
	    joystick_handler.y = 0;
	    //Re-initialize joystick and queue
	    ADC_Init();
	    xQueueReset(queue_joystick);
	}

#ifdef DEBUGGING
	x = joystick_handler.x;
	y = joystick_handler.y;
#endif

	vTaskDelayUntil(&tick, period);
    }
}

void Task_Climbing(void *param) {
    //Initialize rear and back motor
    bd25l_Init(&rearMotor);
    bd25l_Init(&backMotor);
    runMotor(&rearMotor, 0);
    runMotor(&backMotor, 0);
    emBrakeMotor(0);
    while (1) {

#ifdef BUTTON_CONTROL
	/*Button Control*/
	if (BITCHECK(state, 0) && (BITCHECK(state,2) == 0))
	    speed[FRONT_INDEX] = 10;
	else if (BITCHECK(state, 0) && BITCHECK(state, 2))
	    speed[FRONT_INDEX] = 10;
	else if (BITCHECK(state,0) == 0)
	    speed[FRONT_INDEX] = 10;

	if (BITCHECK(state, 1) && (BITCHECK(state,2) == 0))
	    speed[BACK_INDEX] = 10;
	else if (BITCHECK(state, 1) && BITCHECK(state, 2))
	    speed[BACK_INDEX] = 10;
	else if (BITCHECK(state,1) == 0)
	    speed[BACK_INDEX] = 10;

//	if ((state & (1 << 0)) && ((state & (1 << 2)) == 0))
//	    speed[FRONT_INDEX] = 10;
//	else if ((state & (1 << 0)) && (state & (1 << 2)))
//	    speed[FRONT_INDEX] = -10;
//	else if ((state & (1 << 0)) == 0)
//	    speed[FRONT_INDEX] = 0;
//
//	if ((state & (1 << 1)) && ((state & (1 << 2)) == 0))
//	    speed[BACK_INDEX] = 10;
//	else if ((state & (1 << 1)) && (state & (1 << 2)))
//	    speed[BACK_INDEX] = -10;
//	else if ((state & (1 << 1)) == 0)
//	    speed[BACK_INDEX] = 0;
#endif

	//*****VERY IMPORTANT AND MUST NOT BE COMMENTED OUT**********************************//
	//Safety check for to avoid the climbing leg overturn
	if (encoderFront.encoder_pos < FRONT_FULL_ROTATION_ENC / 2) {
	    if (encoderFront.encoder_pos > MAX_FRONT_ALLOWABLE_ENC && speed[FRONT_INDEX] > 0)
		speed[FRONT_INDEX] = 0;
	}
	else {
	    if (encoderFront.encoder_pos < MIN_FRONT_ALLOWABLE_ENC && speed[FRONT_INDEX] < 0)
		speed[FRONT_INDEX] = 0;
	}

	if (encoderBack.encoder_pos < BACK_FULL_ROTATION_ENC / 2) {
	    if (encoderBack.encoder_pos > MAX_BACK_ALLOWABLE_ENC && speed[BACK_INDEX] > 0)
		speed[BACK_INDEX] = 0;
	}
	else {
	    if (encoderBack.encoder_pos < MIN_BACK_ALLOWABLE_ENC && speed[BACK_INDEX] < 0)
		speed[BACK_INDEX] = 0;
	}
	//**********************************************************************************//

	runMotor(&rearMotor, speed[FRONT_INDEX]);
	runMotor(&backMotor, speed[BACK_INDEX]);

	if (speed[FRONT_INDEX] == 0 && speed[BACK_INDEX] == 0)
	    emBrakeMotor(0);
	else
	    emBrakeMotor(1);

	vTaskDelay(10);

    }
}

void Task_USB(void *param) {
    while (1) {
	vTaskDelay(1000);
    }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
