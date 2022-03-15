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
#include "timers.h"

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
#include "tfmini.h"
#include "usb_device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define BUTTON_CONTROL
#define USB_CONTROL
#define ONE_BUTTON_CONTROL_CURB_CLIMBING
//#define USB_CMD_CONTROL

//#define DEBUGGING

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define TO_RAD(x) x * M_PI / 180
#define TO_DEG(x) x * 180 / M_PI

#define BITSET(byte,nbit)   ((byte) |=  (1<<(nbit)))
#define BITCLEAR(byte,nbit) ((byte) &= ~(1<<(nbit)))
#define BITCHECK(byte,nbit) ((byte) & 1<<(nbit))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

const float forward_distance = BASE_LENGTH; // (in meter) distance to travel during climbing process by hub
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

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

extern TaskHandle_t task_control;
extern TaskHandle_t task_keyboard;
extern TaskHandle_t task_normalDrive;
extern TaskHandle_t task_climb_sensor;
extern TaskHandle_t task_joystick;
extern TaskHandle_t task_climbing;
extern TaskHandle_t task_usb;

extern QueueHandle_t queue_joystick;
extern QueueHandle_t encoder;

extern TimerHandle_t timer_buzzer;

extern EncoderHandle encoderBack;
extern EncoderHandle encoderFront;

int speed[2] = {
	0
};
uint8_t touch_down[2] = {
	0
};
speedConfig speed_config = {
	.min_vel = -0.25, 	//Need to be lower than |max_vel| to make sure reverse is slower and safer
	.max_vel = 0.4,	//gear level/100
	.min_acc = -0.8,	//max_vel*2
	.max_acc = 0.2
//max_vel2
};
Motor_TypeDef rearMotor, backMotor; //declare in bd25l.c
uint8_t button_state = 0;

JoystickHandle *joystick_ptr = NULL;
Operation_Mode lifting_mode = RETRACTION;
int x, y;
HAL_StatusTypeDef hub_motor_status;

//Front Climbing Position Control
PID_t frontClimb_pid;
extern float frontClimb_input, frontClimb_output;
extern float frontClimb_setpoint;

//Back Climbing Position Control
PID_t backClimb_pid;
extern float backClimb_input, backClimb_output;
extern float backClimb_setpoint;

extern TickType_t last_hub_rx_t;
extern TickType_t last_can_rx_t[2];
extern TickType_t last_tf_mini_t;

extern uint8_t pBuffer[TFMINI_RX_SIZE];
extern uint8_t usbBuffer[1];
uint32_t back_encoder_input = 0;
uint8_t finish_climbing_flag = 0; //1 if climbing motion finish

uint8_t usb_climb_state = 0;

extern uint8_t buzzer_expiry_count;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void LED_Mode_Configuration(Operation_Mode mode);
bool climbingForward(float dist); //return true if in the process of moving forward
bool goto_pos(int enc, PID_t pid_t); //return true if still in the process of reaching the position
bool in_climb_process(int front_enc, int back_enc);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Task_Control(void *param) {
    while (1) {
	xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
	if (lifting_mode == DANGER) {
	    portENTER_CRITICAL();
	    vTaskSuspendAll();
	    runMotor(&rearMotor, 0);
	    runMotor(&backMotor, 0);
	    HubMotor_SendCommand(0, 0);
	    LED_Mode_Configuration(DANGER);
	    portEXIT_CRITICAL();
	}
    }
}

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
    const TickType_t period = pdMS_TO_TICKS(250); //execution period

    while (1) {
	Button_FilteredInput(&button1, 30);
	Button_FilteredInput(&button2, 30);
	Button_FilteredInput(&button3, 30);

#ifndef USB_CONTROL
	if (button1.state == 1)
	    BITSET(button_state, 0);
	else
	    BITCLEAR(button_state, 0);

	if (button2.state == 1)
	    BITSET(button_state, 1);
	else
	    BITCLEAR(button_state, 1);

	if (button3.state == 1)
	    BITSET(button_state, 2);
	else
	    BITCLEAR(button_state, 2);
#endif
	vTaskDelayUntil(&tick, period);
    }
}

void Task_NormalDrive(void *param) {
    differentialDrive_Handler differential_drive_handler;
    Gear_Level gear_level = GEAR1; //change the speed level if need higher speed
    Sabertooth_Handler sabertooth_handler;

    differentialDriveInit(&differential_drive_handler, FREQUENCY);
    //Initialize base wheel
    MotorInit(&sabertooth_handler, 128, &huart6);
    MotorStartup(&sabertooth_handler);
    MotorStop(&sabertooth_handler);
    while (1) {
	if (lifting_mode == NORMAL) {
	    LED_Mode_Configuration(NORMAL);
	    if (xTimerIsTimerActive(timer_buzzer) == pdTRUE) {
		xTimerStop(timer_buzzer, 1);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
	    }
	    if (joystick_ptr != NULL) {
		computeSpeed(&differential_drive_handler, joystick_ptr->x, joystick_ptr->y, gear_level);
		differentialDrivetoSabertoothOutputAdapter(&differential_drive_handler, &sabertooth_handler);
	    }
	    else {
		//if no joystick data is received, stop both left and right wheel
		MotorThrottle(&sabertooth_handler, 1, 0);
		MotorThrottle(&sabertooth_handler, 2, 0);
	    }
	}
	else if (lifting_mode == STOP) {
	    MotorThrottle(&sabertooth_handler, 1, 0);
	    MotorThrottle(&sabertooth_handler, 2, 0);
//	    if (xTimerIsTimerActive(timer_buzzer) == pdFALSE) {
//		buzzer_expiry_count = 1; //beep for 1 second
//		xTimerStart(timer_buzzer, 1);
//	    }
	    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

	}
	else if (lifting_mode == CURB_DETECTED) {
	    if (joystick_ptr != NULL) {
		joystick_ptr->y = (joystick_ptr->y > 0) ? 0 : joystick_ptr->y;
		computeSpeed(&differential_drive_handler, joystick_ptr->x, joystick_ptr->y, gear_level);
		differentialDrivetoSabertoothOutputAdapter(&differential_drive_handler, &sabertooth_handler);
		if (xTimerIsTimerActive(timer_buzzer) == pdFALSE) {
		    buzzer_expiry_count = 0;
		    xTimerStart(timer_buzzer, 1);
		}
	    }
	}
	else {
	    //If not in driving mode
	    MotorThrottle(&sabertooth_handler, 1, 0);
	    MotorThrottle(&sabertooth_handler, 2, 0);
	    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
	}
	vTaskDelay(10);
    }
}

void Task_Climb_Sensor(void *param) {
    Button_TypeDef rearLS1, rearLS2, backLS1, backLS2;
    memset(&rearLS1, 0, sizeof(Button_TypeDef));
    memset(&rearLS2, 0, sizeof(Button_TypeDef));
    memset(&backLS1, 0, sizeof(Button_TypeDef));
    memset(&backLS2, 0, sizeof(Button_TypeDef));

    rearLS1.gpioPort = LimitSW1_GPIO_Port;
    rearLS1.gpioPin = LimitSW1_Pin;
    rearLS2.gpioPort = LimitSW2_GPIO_Port;
    rearLS2.gpioPin = LimitSW2_Pin;
    backLS1.gpioPort = LimitSW3_GPIO_Port;
    backLS1.gpioPin = LimitSW3_Pin;
    backLS2.gpioPort = LimitSW4_GPIO_Port;
    backLS2.gpioPin = LimitSW4_Pin;

    TickType_t tick = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50); //execution period

    //Initialize sensor and reception
    ENCODER_Init();
    HAL_UART_Receive_DMA(&huart1, pBuffer, TFMINI_RX_SIZE);
    while (1) {
	ENCODER_Get_Angle(&encoderBack);
	ENCODER_Get_Angle(&encoderFront);

	Button_FilteredInput(&rearLS1, 5);
	Button_FilteredInput(&rearLS2, 5);
	Button_FilteredInput(&backLS1, 5);
	Button_FilteredInput(&backLS2, 5);

	if (rearLS1.state == 1 || rearLS2.state == 1)
	    touch_down[FRONT_INDEX] = 1;
	else
	    touch_down[FRONT_INDEX] = 0;

	if (backLS1.state == 1 || backLS2.state == 1)
	    touch_down[BACK_INDEX] = 1;
	else
	    touch_down[BACK_INDEX] = 0;

	if ((xTaskGetTickCount() - last_can_rx_t[0]) > 2000 || (xTaskGetTickCount() - last_can_rx_t[1]) > 2000) {
	    lifting_mode = DANGER;
	    xTaskNotify(task_control, 0, eNoAction);
	}

	if (xTaskGetTickCount() - last_tf_mini_t > 200) {
	    HAL_UART_Receive_DMA(&huart1, pBuffer, TFMINI_RX_SIZE);
	}

	vTaskDelayUntil(&tick, period);
    }
}

void Task_Joystick(void *param) {

    //Note that the period cannot be shorter than 100
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
	joystick_ptr = &joystick_handler;
	vTaskDelayUntil(&tick, period);
    }
}

void Task_Climbing(void *param) {
    //Store user press button state and climb iteration
    uint8_t button_prev_state = 0;
    uint8_t climb_first_iteration = 1; //1 if 1st climb iteration

    Operation_Mode dummy_mode = EMPTY; //to store climbing up or down state

    uint32_t front_climbDown_enc = 0;

    //Initialize hub motor
	HubMotor_Init()

    //Initialize rear and back motor to zero and engage the brake
    bd25l_Init(&rearMotor);
    bd25l_Init(&backMotor);
    runMotor(&rearMotor, 0);
    runMotor(&backMotor, 0);
    emBrakeMotor(0);

    while (1) {

#ifdef BUTTON_CONTROL
	/*Button Control*/
	if (BITCHECK(button_state, 0) && (BITCHECK(button_state,2) == 0))
	    speed[FRONT_INDEX] = 10;
	else if (BITCHECK(button_state, 0) && BITCHECK(button_state, 2))
	    speed[FRONT_INDEX] = -10;
	else if (BITCHECK(button_state,0) == 0)
	    speed[FRONT_INDEX] = 0;

	if (BITCHECK(button_state, 1) && (BITCHECK(button_state,2) == 0))
	    speed[BACK_INDEX] = 10;
	else if (BITCHECK(button_state, 1) && BITCHECK(button_state, 2))
	    speed[BACK_INDEX] = -10;
	else if (BITCHECK(button_state,1) == 0)
	    speed[BACK_INDEX] = 0;

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

#ifdef ONE_BUTTON_CONTROL_CURB_CLIMBING
	//when button3 is pressed, Extend climbing wheel until both wheel touches the ground
	if (((BITCHECK(button_state,2) || button_prev_state == 1) || usb_climb_state == 1)
		&& climb_first_iteration == 1) {
	    button_prev_state = 1;
	    //Put both leg to same initial position for easier curb climbing mode detection
	    if (abs(encoderFront.signed_encoder_pos) >= 50 || abs(encoderBack.signed_encoder_pos) >= 50) {
		goto_pos(0, frontClimb_pid);
		goto_pos(0, backClimb_pid);
		lifting_mode = EMPTY;
	    }
	    else {
		speed[FRONT_INDEX] = 0;
		speed[BACK_INDEX] = 0;
		lifting_mode = LANDING;
		button_prev_state = 0;
		usb_climb_state = 0;
		vTaskDelay(pdMS_TO_TICKS(300));
	    }
	}

	//Landing both leg until both touches the ground
	//Touch down is used to indicate whether the Limit switch on the leg has triggered
	if ((touch_down[FRONT_INDEX] == 0 || touch_down[BACK_INDEX] == 0) && lifting_mode == LANDING) {
	    //Disengage the motor brake
	    emBrakeMotor(1);

	    //if front touch before back, climbing up process
	    if (touch_down[BACK_INDEX] == 0 && touch_down[FRONT_INDEX] == 1 && lifting_mode == LANDING)
		dummy_mode = CLIMB_UP;
	    //if back touch before front, climbing down process
	    else if (touch_down[BACK_INDEX] == 1 && touch_down[FRONT_INDEX] == 0 && lifting_mode == LANDING)
		dummy_mode = CLIMB_DOWN;

	    //continue to move the leg until it touches ground
	    if (touch_down[BACK_INDEX] == false)
		speed[BACK_INDEX] = 30;
	    else
		speed[BACK_INDEX] = 0;

	    if (touch_down[FRONT_INDEX] == false)
		speed[FRONT_INDEX] = 30;
	    else
		speed[FRONT_INDEX] = 0;
	}

	//When both leg touches ground
	if ((touch_down[FRONT_INDEX] == 1 && touch_down[BACK_INDEX] == 1) && lifting_mode == LANDING) {
	    speed[FRONT_INDEX] = 0;
	    speed[BACK_INDEX] = 0;
	    emBrakeMotor(0);
	    lifting_mode = dummy_mode;
	    vTaskDelay(50); //Add delay to ensure the legs lose its inertia
	}

	//Climbing up process
	if (lifting_mode == CLIMB_UP) {
	    LED_Mode_Configuration(CLIMB_UP);

	    if (climb_first_iteration) {
		//If curb_height is positive, should be climbing up process and vice versa
		float curb_height = CLIMBING_LEG_LENGTH * cos(TO_RAD(encoderFront.angleDeg)) + BASE_HEIGHT
			- FRONT_CLIMB_WHEEL_DIAMETER / 2.0;
		curb_height += 0.015; //Small error correction 10%

		//First determine whether is the height climb-able
		float back_lifting_height = BACK_BASE_HEIGHT + curb_height - HUB_DIAMETER / 2;
		float back_lifting_angle =
		TO_DEG(
			(float )acos(
				-back_lifting_height
				/ CLIMBING_LEG_LENGTH)) - 30.0; //30.0 is the bending angle of the extender(originally 36.6).
		back_encoder_input = (back_lifting_angle / 360.0) * (4096 * BACK_GEAR_RATIO);

		//3 different scenerio to abort the climbing up task
		//1. The angle calculated is not feasible
		//2. The leg rotate more than it supposed to
		//3. The curb height is too low where climbing up is unnecessary
		if (isnan(back_lifting_angle) || back_encoder_input >= MAX_BACK_ALLOWABLE_ENC || curb_height <= 0.05) {
		    lifting_mode = RETRACTION;
		    continue;
		}
		speed[BACK_INDEX] = 0;
		speed[FRONT_INDEX] = 0;
		climb_first_iteration = false;
	    }
	    //Mathematical Model
	    //Start Climbing process
	    if (finish_climbing_flag == false) {
		if (!in_climb_process(MAX_FRONT_CLIMBING_ENC, back_encoder_input))
		    finish_climbing_flag = true;
	    }

	    if (finish_climbing_flag == true) {
		emBrakeMotor(0);
		if (!(climbingForward(forward_distance))) {
		    emBrakeMotor(1);
		    finish_climbing_flag = false;
		    lifting_mode = RETRACTION;
		    vTaskDelay(pdMS_TO_TICKS(100));
		}
	    }
	}

	else if (lifting_mode == CLIMB_DOWN) {
	    LED_Mode_Configuration(CLIMB_DOWN);
	    if (climb_first_iteration) {
		front_climbDown_enc = encoderFront.encoder_pos + 5.0 / 360.0 * 4096 * FRONT_GEAR_RATIO;

		//First determine whether is the height climb-able
		if (front_climbDown_enc > MAX_FRONT_ALLOWABLE_ENC) {
		    lifting_mode = RETRACTION;
		    continue;
		}
		climb_first_iteration = false;

		speed[BACK_INDEX] = 0;
		speed[FRONT_INDEX] = 0;
	    }

	    //Start Climbing process
	    if (finish_climbing_flag == false) {
		if (!in_climb_process(front_climbDown_enc, MAX_BACK_CLIMBING_ENC))
		    finish_climbing_flag = true;
	    }

	    if (finish_climbing_flag == true) {
		emBrakeMotor(0);
		if (!(climbingForward(forward_distance))) {
		    emBrakeMotor(1);
		    finish_climbing_flag = false;
		    lifting_mode = RETRACTION;
		    vTaskDelay(pdMS_TO_TICKS(100));
		}
	    }
	}

	if (lifting_mode == RETRACTION) {
	    LED_Mode_Configuration(RETRACTION);
	    //retraction process
	    if (abs(encoderBack.encoder_pos - (MIN_BACK_ALLOWABLE_ENC)) > 30
		    || abs(encoderFront.encoder_pos - (MIN_FRONT_ALLOWABLE_ENC)) > 30) {
		goto_pos(MIN_BACK_ALLOWABLE_ENC, backClimb_pid);
		goto_pos(MIN_FRONT_ALLOWABLE_ENC, frontClimb_pid);
		if (speed[FRONT_INDEX] == 0 && speed[BACK_INDEX] == 0)
		    lifting_mode = NORMAL;
	    }
	    else {
		pid_reset(frontClimb_pid);
		pid_reset(backClimb_pid);
		lifting_mode = NORMAL;
	    }

	}

#endif

#ifdef USB_CMD_CONTROL

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
	//If lifting mode is normal, notify driving task
	if (lifting_mode == NORMAL) {
	    speed[FRONT_INDEX] = 0;
	    speed[BACK_INDEX] = 0;
	    climb_first_iteration = 1;
	    xTaskNotify(task_normalDrive, 0, eNoAction);
	}
	//**********************************************************************************//

	runMotor(&rearMotor, speed[FRONT_INDEX]);
	runMotor(&backMotor, speed[BACK_INDEX]);

	if (speed[FRONT_INDEX] == 0 && speed[BACK_INDEX] == 0)
	    emBrakeMotor(0);
	else
	    emBrakeMotor(1);

	//Note that this time is critical to smooth climbing execution
	vTaskDelay(100);
    }
}

void Task_USB(void *param) {
    while (1) {
	if (usbBuffer[0] == USB_STOP) {
	    lifting_mode = STOP;
	    xTaskNotify(task_normalDrive, 0, eNoAction);
	}
	else if (usbBuffer[0] == USB_MOVE) {
	    lifting_mode = NORMAL;
	    xTaskNotify(task_normalDrive, 0, eNoAction);
	}
	else if (usbBuffer[0] == USB_CURB_DETECTED) {
	    lifting_mode = CURB_DETECTED;
	    xTaskNotify(task_normalDrive, 0, eNoAction);
	}
	else if (usbBuffer[0] == USB_CLIMB_UP || usbBuffer[0] == USB_CLIMB_DOWN) {
	    usb_climb_state = 1;
	}
	usbBuffer[0] = 0;

	vTaskDelay(1000);

    }
}

void LED_Mode_Configuration(Operation_Mode mode) {
    switch (mode) {
	case NORMAL:
	    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	    break;
	case CLIMB_UP:
	    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	    break;
	case CLIMB_DOWN:
	    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	    break;
	case RETRACTION:
	case DANGER:
	    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	default:
	    break;
    }
}

//Hub motor move forward  by preset dist
bool climbingForward(float dist) {
    static uint32_t prev_tick = 0;
    static int32_t prev_enc;
    static bool first_loop = true;
    static float dist_remaining;

    float rps = (dist >= 0) ? 1.0 : -1.0; //rad/s

    if (first_loop) {
	prev_enc = hub_encoder_feedback.encoder_2;
	prev_tick = HAL_GetTick();
	first_loop = false;
	dist_remaining = dist;
    }
    if (xTaskGetTickCount() - last_hub_rx_t > 1500) {
	lifting_mode = DANGER;
	xTaskNotify(task_control, 0, eNoAction);
	return false;
    }
    if (dist / dist_remaining >= 0 && first_loop == false) {
	hub_motor_status = HubMotor_SendCommand(rps, rps);
	if (HAL_GetTick() - prev_tick > 1) {
	    float dt = (float) (HAL_GetTick() - prev_tick) / FREQUENCY;
	    float rad_per_s = ((float) (hub_encoder_feedback.encoder_2 - prev_enc) / dt) * 2 * M_PI / 4096;
	    dist_remaining -= (HUB_DIAMETER * rad_per_s * dt) / 2;
	    prev_tick = HAL_GetTick();
	    prev_enc = hub_encoder_feedback.encoder_2;
	}
	return true;
    }
    else {
	first_loop = true;
	hub_motor_status = HubMotor_SendCommand(0, 0);
	return false;
    }
}

//Control rear and back wheel to set encoder position using PID controller
bool goto_pos(int enc, PID_t pid_t) {
    int cur_enc_pos;
    if (pid_t == frontClimb_pid) {
	cur_enc_pos = (int) encoderFront.encoder_pos;
	if (pid_need_compute(frontClimb_pid) && fabs(enc - cur_enc_pos) > 5) {
	    // Read process feedback
	    if (cur_enc_pos > MAX_FRONT_ALLOWABLE_ENC)
		cur_enc_pos -= 4096 * FRONT_GEAR_RATIO;
	    if (enc >= MAX_FRONT_ALLOWABLE_ENC)
		enc -= 4096 * FRONT_GEAR_RATIO;
	    frontClimb_setpoint = enc;
	    frontClimb_input = cur_enc_pos;
	    // Compute new PID output value
	    pid_compute(frontClimb_pid);
	    //Change actuator value
	    speed[FRONT_INDEX] = frontClimb_output;
	    if (fabs(speed[FRONT_INDEX]) < 5) {
		speed[FRONT_INDEX] = 0;
		pid_reset(frontClimb_pid);
	    }

	    return true;

	}
	else {
//	    speed[FRONT_INDEX] = 0;
	    return false;
	}
    }
    else if (pid_t == backClimb_pid) {
	cur_enc_pos = (int) encoderBack.encoder_pos;
	if (pid_need_compute(backClimb_pid) && fabs(enc - cur_enc_pos) > 5) {
	    // Read process feedback
	    //following code is causing back turn
	    if (cur_enc_pos > MAX_BACK_ALLOWABLE_ENC)
		cur_enc_pos -= 4096 * BACK_GEAR_RATIO;
	    if (enc >= MAX_BACK_ALLOWABLE_ENC)
		enc -= 4096 * BACK_GEAR_RATIO;
	    backClimb_setpoint = enc;
	    backClimb_input = cur_enc_pos;
	    // Compute new PID output value
	    pid_compute(backClimb_pid);
	    //Change actuator value
	    speed[BACK_INDEX] = backClimb_output;
	    if (fabs(speed[BACK_INDEX]) < 5) {
		speed[BACK_INDEX] = 0;
		pid_reset(backClimb_pid);
	    }
	    return true;
	}
	else {
//			speed[BACK_INDEX] = 0;
//			if(fabs(speed[BACK_INDEX]) < 5)
//				pid_reset(backClimb_pid);
	    return false;
	}
    }
    return false;
}

//Lifting process with desired front and back encoder position
bool in_climb_process(int front_enc, int back_enc) {
    bool is_lifting;
    static bool first_loop = true;
    static uint32_t prev_angle_tick = 0;
    static float prev_angle = 0;
    float climbForward_speed = 0;

    if (first_loop) {
	prev_angle = encoderBack.angleDeg;
	prev_angle_tick = HAL_GetTick();
	first_loop = false;
    }

//1. lift the front climbing wheel up until it reach it maximum pos
//The process is controlled by PID on the front climbing wheel
//the maximum pos is when the climbing wheel is below the wheelchair base
    goto_pos(front_enc, frontClimb_pid);
    goto_pos(back_enc, backClimb_pid);

    if (fabs(speed[FRONT_INDEX]) >= 5 || fabs(speed[BACK_INDEX]) >= 5)
	is_lifting = true;
    else
	is_lifting = false;

//2. During lifting, due to fixed point at the back climbing wheel.
//The wheelchair would be pulled back if the back wheel not traveling while the its lifting
//Therefore, lifting of back wheel and hub motor need to work at the same time to make sure the wheelchair is not moving back.
//Pull back of wheelchair would cause the front climbing wheel to slip from the curb
    if (is_lifting == true && speed[BACK_INDEX] != 0) {
	double dt = (HAL_GetTick() - prev_angle_tick) / (float) FREQUENCY;
	climbForward_speed = CLIMBING_LEG_LENGTH * (sin(TO_RAD(prev_angle)) - sin(TO_RAD(encoderBack.angleDeg))) / dt; //unit: m/s,
	climbForward_speed = climbForward_speed / (HUB_DIAMETER / 2);
	//Convert hub speed into pulse/second
	hub_motor_status = HubMotor_SendCommand(climbForward_speed, climbForward_speed);
	prev_angle = encoderBack.angleDeg;
	prev_angle_tick = HAL_GetTick();
    }
    else if (is_lifting == true && speed[BACK_INDEX] == 0)
	hub_motor_status = HubMotor_SendCommand(0, 0);

    if (!is_lifting)
	first_loop = true;
    return is_lifting;

}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
