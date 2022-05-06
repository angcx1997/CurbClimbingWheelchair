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
#include "semphr.h"
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
#include "battery.h"
//#include "wheelchair.h"
#include "PID.h"
#include "Sabertooth.h"
#include "joystick.h"
#include "differentialDrive.h"
#include "DifferentialDrivetoSabertooth.h"
#include "briter_encoder_rs485.h"
#include "tfmini.h"
#include "usb_device.h"
#include "encoder_util.h"
#include "../script/DataLogger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    TickType_t max_tick;
    TickType_t new_tick;
    TickType_t avg_tick;
    TickType_t sum_tick;
    uint32_t count;
} Debug_Tick_t;

typedef struct {
    uint8_t button1 :1;
    uint8_t button2 :1;
    uint8_t button3 :1;
} button_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef ULONG_MAX
    #define ULONG_MAX 0xFFFFFFFF
#endif
//#define DEBUGGING
//#define DATA_LOGGING
#ifndef MAX
    #define MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif
#ifndef MIN
    #define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif
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
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

extern TaskHandle_t task_control;
extern TaskHandle_t task_keyboard;
extern TaskHandle_t task_normalDrive;
extern TaskHandle_t task_climb_sensor;
extern TaskHandle_t task_navigation_sensor;
extern TaskHandle_t task_joystick;
extern TaskHandle_t task_climbing;
extern TaskHandle_t task_usb;

extern QueueHandle_t queue_joystick_raw;
extern QueueHandle_t encoder;

extern xSemaphoreHandle mutex_joystick;

extern TimerHandle_t timer_buzzer;

extern EncoderHandle encoderBack;
extern QueueHandle_t queue_battery_level;
extern EncoderHandle encoderFront;

/*Used to store climbing leg speed*/
int speed[2] = {
	0
};
/*Used to tell if both same-sided leg is contact with ground*/
uint8_t touch_down[2] = {
	0
};
speedConfig speed_config = {
	.min_vel = -0.25, 	//Need to be lower than |max_vel| to make sure reverse is slower and safer
	.max_vel = 0.4,		//gear level/100
	.min_acc = -0.8,	//max_vel*2
	.max_acc = 0.2
//max_vel2
};

//declare in bd25l.c
Motor_TypeDef rearMotor, backMotor;

/*Used to store button state of 3 different button*/
button_state_t button_state = {
	0
};

JoystickHandle *joystick_ptr = NULL;
Operation_Mode lifting_mode = RETRACTION;
#ifdef DEBUGGING
int x, y;
#endif
/*Status of hub uart sending data*/
HAL_StatusTypeDef hub_motor_status;

/*Front climb position control PID variable*/
PID_t frontClimb_pid;
extern float frontClimb_input, frontClimb_output;
extern float frontClimb_setpoint;

/*Back climb position control PID variable*/
PID_t backClimb_pid;
extern float backClimb_input, backClimb_output;
extern float backClimb_setpoint;

/*Used to check if the transmission happen in time*/
extern TickType_t last_hub_rx_t;
extern TickType_t last_can_rx_t[2];
extern TickType_t last_rs485_enc_t[2];
extern TickType_t last_tf_mini_t;

extern uint8_t tf_rx_buf[TFMINI_RX_SIZE];
extern uint8_t usbBuffer[1];
uint32_t back_encoder_input = 0;
uint8_t finish_climbing_flag = 0; //1 if climbing motion finish

uint8_t usb_climb_state = 0;

//Base wheel control
extern Briter_Encoder_t base_encoder[2];
Debug_Tick_t encoder_tick_taken = {
	0
};

extern wheel_velocity_t base_velocity[2];

MPU6050_t MPU6050;

#ifdef DATA_LOGGING
DataLogger_Msg_t datalog_msg;
#endif

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void LED_Mode_Configuration(Operation_Mode mode);
static bool climbingForward(float dist); //return true if in the process of moving forward
static bool goto_pos(int enc, PID_t pid_t); //return true if still in the process of reaching the position
static bool in_climb_process(int front_enc, int back_enc);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Task_Control(void *param) {
    /*
     * Pre-empt all other task and immediately stop running wheel
     */
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

/**
 * Button Interface
 */
void Task_Keyboard(void *param) {
    /*
     * Use to store user button state
     * Button 0 (leftmost)	: Get out of stop mode
     * Button 1	(middle)	:
     * Button 2 (rightmost)	: Go into climbing mode
     */

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

	button_state.button1 = (button1.state == 1) ? 1 : 0;
	button_state.button2 = (button2.state == 1) ? 1 : 0;
	button_state.button3 = (button3.state == 1) ? 1 : 0;

	//Use button 1 to get out of STOP mode and enter into curb detected
	//Where the system is only allowed to move backward
	if (button_state.button1 == 1 && lifting_mode == STOP) {
	    lifting_mode = CURB_DETECTED;
	    xTaskNotify(task_normalDrive, 0, eNoAction);
	}
	//Use button 2 to get out of STOP mode and enter into curb detected
	//Where the system is only allowed to move backward
	if (button_state.button2 == 1) {
	    lifting_mode = DANGER;
	    xTaskNotify(task_control, 0, eNoAction);
	}
	vTaskDelayUntil(&tick, period);
    }
}

void Task_NormalDrive(void *param) {
    /*
     * Mainly control wheelchair base wheel, task run when normal driving
     */
    differentialDrive_Handler differential_drive_handler;
    Gear_Level gear_level = GEAR1; //change the speed level if need higher speed
    Sabertooth_Handler sabertooth_handler;
    //declare a struct to hold linear and angular velocity
    struct Command_Velocity {
	float linear;
	float angular;
    };
    struct Command_Velocity cmd_vel = {
	    0, 0
    };

    //Initialize base wheel
    memset(&base_velocity[LEFT_INDEX], 0, sizeof(wheel_velocity_t));
    memset(&base_velocity[RIGHT_INDEX], 0, sizeof(wheel_velocity_t));
    DDrive_Init(&differential_drive_handler, FREQUENCY);
    MotorInit(&sabertooth_handler, 128, &huart6);
    MotorStartup(&sabertooth_handler);
    MotorStop(&sabertooth_handler);

    //Battery level
    uint16_t voltage_level = 25;


#ifdef DATA_LOGGING
    DataLogger_Init(&datalog_msg);
    uint32_t tick_count = 0;
#endif
    while (1) {
	if (lifting_mode == NORMAL) {
	    /*Data Processing*/
//	    	TickType_t start_t = xTaskGetTickCount();
//	    	float tmp = calculateVelocity(&left_wheel, base_encoder[0].encoder_value);
//	    	float tmp1 = calculateVelocity(&right_wheel, base_encoder[1].encoder_value);
//	    	TickType_t total_t = xTaskGetTickCount();


	    //When user in normal driving mode
	    LED_Mode_Configuration(NORMAL);

	    //Stop buzzer timer if user mode has return to normal
	    if (xTimerIsTimerActive(timer_buzzer) == pdTRUE) {
		xTimerStop(timer_buzzer, 1);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
	    }

	    //Calculate wheel velocity from joystick input
	    if (joystick_ptr == NULL) {
		MotorStop(&sabertooth_handler);
	    }
	    else {
		cmd_vel.linear = joystick_ptr->y;
		cmd_vel.angular = joystick_ptr->x;
	    }
	}
	else if (lifting_mode == CURB_DETECTED) {
	    //Use mutex to safeguard joystick ptr from being modified in Task_Joystick
	    if (xSemaphoreTake(mutex_joystick, pdMS_TO_TICKS(5)) == pdTRUE) {
		//When curb is detected by distance sensor,
		//Restrict wheelchair movement to prevent user move forward
		if (joystick_ptr != NULL) {
		    joystick_ptr->y = (joystick_ptr->y > 0) ? 0 : joystick_ptr->y;
		    cmd_vel.linear = joystick_ptr->y;
		    cmd_vel.angular = joystick_ptr->x;
		}
		xSemaphoreGive(mutex_joystick);
	    }
	    //Notify user that he is in danger situation
	    //Function like beeping sound when cars is reversing
	    if (xTimerIsTimerActive(timer_buzzer) == pdFALSE) {
		xTimerStart(timer_buzzer, 1);

	    }
	}
	else if (lifting_mode == STOP) {
	    //When front distance sensor
	    MotorStop(&sabertooth_handler);
	    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
	    continue;
	}
	else {
	    //If not in driving mode
	    MotorStop(&sabertooth_handler);
	    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
	    continue;
	}

	//Receive voltage from queue battery
	if(uxQueueMessagesWaiting(queue_battery_level)){
	    xQueueReceive(queue_battery_level, &voltage_level, 0);
	}

//	DDrive_SpeedMapping(&differential_drive_handler, cmd_vel.angular, cmd_vel.linear, gear_level);
//	dDriveToST_Adapter(&differential_drive_handler, &sabertooth_handler);
	//TODO: Add PID controller here
#ifdef DATA_LOGGING
	/*============================================================================*/
	/*Input sine wave*/
//	uint16_t sampling_rate = 750;
//	//generate sine wave
//	float v = 0.75 * sin(2 * M_PI * tick_count/sampling_rate);
//	tick_count++;
//
//	int motor_output_1 = v * SABERTOOTH_MAX_ALLOWABLE_VALUE;
//	int motor_output_2 = v * SABERTOOTH_MAX_ALLOWABLE_VALUE;
//	MotorThrottle(&sabertooth_handler, TARGET_2, motor_output_1);
//	MotorThrottle(&sabertooth_handler, TARGET_1, motor_output_2);
//
//	LOG_battery = voltage_level;
//	LOG_left_vel = motor_output_1;
//	LOG_right_vel = motor_output_2;
	/*============================================================================*/
	/*Joystick input*/
    	DDrive_SpeedMapping(&differential_drive_handler, cmd_vel.angular, cmd_vel.linear, gear_level);
    	dDriveToST_Adapter(&differential_drive_handler, &sabertooth_handler);

    	//Declare variable
    	uint16_byte_u LOG_battery;
    	float_byte_u LOG_left_pwm;
    	float_byte_u LOG_right_pwm;
    	uint32_byte_u LOG_left_enc;
    	uint32_byte_u LOG_right_enc;
    	float_byte_u LOG_left_enc_vel;
    	float_byte_u LOG_right_enc_vel;
    	uint32_byte_u LOG_tick;

    	//Define variable
//    	LOG_battery.data = voltage_level;
//    	LOG_left_pwm.data = differential_drive_handler.m_leftMotor;
//    	LOG_right_pwm.data = differential_drive_handler.m_rightMotor;
//    	LOG_left_enc.data = base_encoder[LEFT_INDEX].encoder_value;
//    	LOG_right_enc.data = base_encoder[RIGHT_INDEX].encoder_value;
//    	LOG_left_enc_vel.data = base_velocity[LEFT_INDEX].angular_velocity;
//    	LOG_right_enc_vel.data = base_velocity[RIGHT_INDEX].angular_velocity;
    	LOG_battery.data = 0xABCD;
	LOG_left_pwm.data = 0.54321;
	LOG_right_pwm.data = 0.56789;
	LOG_left_enc.data = 0x12345670;
	LOG_right_enc.data = 0x89ABCDEF;
	LOG_left_enc_vel.data = 0.12345;
	LOG_right_enc_vel.data = 0.67890;
    	LOG_tick.data = xTaskGetTickCount();

    	DataLogger_AddMessage(&datalog_msg, LOG_battery.array, sizeof(LOG_battery.array));
    	DataLogger_AddMessage(&datalog_msg, LOG_left_pwm.array, sizeof(LOG_left_pwm.array));
    	DataLogger_AddMessage(&datalog_msg, LOG_right_pwm.array, sizeof(LOG_right_pwm.array));
    	DataLogger_AddMessage(&datalog_msg, LOG_left_enc.array, sizeof(LOG_left_enc.array));
    	DataLogger_AddMessage(&datalog_msg, LOG_right_enc.array, sizeof(LOG_right_enc.array));
	DataLogger_AddMessage(&datalog_msg, LOG_left_enc_vel.array, sizeof(LOG_left_enc_vel.array));
	DataLogger_AddMessage(&datalog_msg, LOG_right_enc_vel.array, sizeof(LOG_right_enc_vel.array));
	DataLogger_AddMessage(&datalog_msg, LOG_tick.array, sizeof(LOG_tick.array));

	//4. After added all the message, call complete tx message to add check to ensure data integrity
	DataLogger_CompleteTxMessage(&datalog_msg);
	CDC_Transmit_FS(datalog_msg.pMsg, datalog_msg.size);

	/*============================================================================*/
#else
    	DDrive_SpeedMapping(&differential_drive_handler, cmd_vel.angular, cmd_vel.linear, gear_level);
	dDriveToST_Adapter(&differential_drive_handler, &sabertooth_handler);
#endif
	vTaskDelay(10);
    }
}

/**
 * This task will used to managed the data acquisition of sensor used for
 * - Climbing
 * 	1. Limit switch
 * 	2. Encoder
 * - Navigation
 * 	1. IMU
 * 	2. Encoder for base wheel
 * - Common
 * 	1. Curb detection sensor
 * Each session is divided into data acquisition and error handling
 */
void Task_Sensor(void *param) {
    /*
     * Climbing
     * Following sensor are highly critical. Single malfunction will cause the climbing motion to be failed
     * Always make sure all sensor is in good condition
     * Stop the climbing process if sensor fault
     */

    //Limit switch located on each individual climbing leg
    //Initialize member
    Button_TypeDef rearLS1 = {
	    0
    }, rearLS2 = {
	    0
    }, backLS1 = {
	    0
    }, backLS2 = {
	    0
    };
    rearLS1.gpioPort = LimitSW1_GPIO_Port;
    rearLS1.gpioPin = LimitSW1_Pin;
    rearLS2.gpioPort = LimitSW2_GPIO_Port;
    rearLS2.gpioPin = LimitSW2_Pin;
    backLS1.gpioPort = LimitSW3_GPIO_Port;
    backLS1.gpioPin = LimitSW3_Pin;
    backLS2.gpioPort = LimitSW4_GPIO_Port;
    backLS2.gpioPin = LimitSW4_Pin;

    //Initialize climbing encoder sensor and start front distance sensor data reception
    ENCODER_Init();
    HAL_UART_Receive_DMA(&huart1, tf_rx_buf, TFMINI_RX_SIZE);

    //    //Reset encoder position
    //    ENCODER_Set_ZeroPosition(&encoderBack);
    //    ENCODER_Set_ZeroPosition(&encoderFront);

    /*
     * Navigation
     * -Encoder
     * 	To keep track of wheel velocity
     */

    //Initialize encoder sensor for base wheel
    BRITER_RS485_Init(&(base_encoder[0]), 0x02, &huart4);
    BRITER_RS485_Init(&(base_encoder[1]), 0x01, &huart4);
    //Flag to alternate the address of rs485 tx to avoid congestion
    uint8_t base_encoder_tx_flag = 0;

    TickType_t debug = xTaskGetTickCount();
    uint32_t state_count = xTaskGetTickCount();
    while (MPU6050_Init(&hi2c1) != HAL_OK)
    {
	if (xTaskGetTickCount() - state_count > 50)
		Error_Handler();
    }
    debug = xTaskGetTickCount() - debug;

    debug = xTaskGetTickCount();
    MPU6050_Calculate_Error(&hi2c1, &MPU6050);
    debug = xTaskGetTickCount() - debug;
    //Ensure periodic execution
    const TickType_t period = pdMS_TO_TICKS(10);
    while (1) {
	//******************************************************************************
	/*Climbing Sensor Data Acquisition and Processing*/
	/*Data Acquisition*/
	//Read encoder data
	ENCODER_Get_Angle(&encoderBack);
	ENCODER_Get_Angle(&encoderFront);

	//Read limit switch state
	Button_FilteredInput(&rearLS1, 5);
	Button_FilteredInput(&rearLS2, 5);
	Button_FilteredInput(&backLS1, 5);
	Button_FilteredInput(&backLS2, 5);

	/*Data Processing*/
	//TODO: What if leg is suspended on the air, touchdown will turn back to 0 and cause the leg to continue landing
	//If both front LS is touched, store in touchdown (use to tell front leg are both in contact with ground)
	if (rearLS1.state == 1 || rearLS2.state == 1)
	    touch_down[FRONT_INDEX] = 1;
	else
	    touch_down[FRONT_INDEX] = 0;

	//If both back LS is touched, store in touchdown (use to tell back leg are both in contact with ground)
	if (backLS1.state == 1 || backLS2.state == 1)
	    touch_down[BACK_INDEX] = 1;
	else
	    touch_down[BACK_INDEX] = 0;

	/*Error Handling*/
	//If encoder is faulty, no data reception, suspend all task
	if ((xTaskGetTickCount() - last_can_rx_t[0]) > 1000 || (xTaskGetTickCount() - last_can_rx_t[1]) > 1000) {
	    lifting_mode = DANGER;
	    xTaskNotify(task_control, 0, eNoAction);
	}

	//******************************************************************************
	/*Navigation sensor data acquisition*/
	/*Data Acquisition*/
	if (base_encoder_tx_flag % 2 == 0) {
	    BRITER_RS485_GetValue_DMA_TX(&base_encoder[0]);
	    base_encoder_tx_flag ++;
	}else
	{
	    BRITER_RS485_GetValue_DMA_TX(&base_encoder[1]);
	    base_encoder_tx_flag --;
	}

	BRITER_RS485_GetValue_DMA_RX(base_encoder);

	/*Error Handling*/
	//If encoder is faulty, no data reception, suspend all task
	if ((xTaskGetTickCount() - last_rs485_enc_t[0]) > 500 || (xTaskGetTickCount() - last_rs485_enc_t[1]) > 500) {
	    lifting_mode = DANGER;
	    xTaskNotify(task_control, 0, eNoAction);
	}
	//******************************************************************************
	/*Common Sensor*/
	//Distance sensor data acquisition is managed by DMA
	//Error Handling
	//UART error cause by noise flag, framing, overrun error happens during multibuffer dma communication
	if (xTaskGetTickCount() - last_tf_mini_t > 500) {
	    lifting_mode = DANGER;
	    xTaskNotify(task_control, 0, eNoAction);
	}



	vTaskDelay(period);
    }
}

/**
 * Joystick interface
 * For acquisition of joystick data
 */
void Task_Joystick(void *param) {

    //Warning: Period cannot be shorter than 100
    TickType_t tick = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100); //execution period

    JoystickHandle joystick_handler;

    //Initialize AD7606
    ADC_Init();

    while (1) {
	ADC_DataRequest();
	//Use mutex to safeguard joystick data being accessed and modified by task_normal_drive
	if (xSemaphoreTake(mutex_joystick, pdMS_TO_TICKS(5)) == pdTRUE) {
	    if(xTaskNotifyWait(0x00, ULONG_MAX, 0, 50) == pdTRUE){
		int16_t adc_rawData[8];
		ADC_Read(adc_rawData);
		joystick_handler.x = adc_rawData[2];
		joystick_handler.y = adc_rawData[1];
		Joystick_CalculatePos(&joystick_handler);
	    }
//	    if (xQueueReceive(queue_joystick_raw, &(joystick_handler), 50) == pdPASS) {
//		Joystick_CalculatePos(&joystick_handler);
//	    }
	    else {
		//If no data is available, force reading to zero
		joystick_handler.x = 0;
		joystick_handler.y = 0;
		//Re-initialize joystick and queue
		ADC_Init();
		xQueueReset(queue_joystick_raw);
	    }

	    joystick_ptr = &joystick_handler;
	    xSemaphoreGive(mutex_joystick);
	}

	vTaskDelayUntil(&tick, period);
    }
}

/**
 * Climbing:
 * Focus on the climbing only
 * 1. Manual control
 * 2. Semi-autonomous climbing control
 * 3. (in progress)fully autonomous control
 */
void Task_Climbing(void *param) {
    //Store user press button state and climb iteration
    uint8_t button_prev_state = 0;
    uint8_t climb_first_iteration = 1; //1 if 1st climb iteration

    Operation_Mode dummy_mode = EMPTY; //to store climbing up or down state

    uint32_t front_climbDown_enc = 0;

    //Initialize hub motor
    HubMotor_Init();

    //Initialize rear and back motor to zero and engage the brake
    bd25l_Init(&rearMotor);
    bd25l_Init(&backMotor);
    runMotor(&rearMotor, 0);
    runMotor(&backMotor, 0);
    emBrakeMotor(0);

    while (1) {

#ifdef BUTTON_CONTROL
	/*Button Control*/
	if (button_state.button1 == 1 && button_state.button3 == 0)
	    speed[FRONT_INDEX] = 10;
	else if (button_state.button1 == 1 && button_state.button3 == 1)
	    speed[FRONT_INDEX] = -10;
	else if (button_state.button1 == 0)
	    speed[FRONT_INDEX] = 0;

	if (button_state.button2 == 1 && button_state.button3 == 0)
	    speed[BACK_INDEX] = 10;
	else if (button_state.button2 == 1 && button_state.button3 == 1)
	    speed[BACK_INDEX] = -10;
	else if (button_state.button2 == 0)
	    speed[BACK_INDEX] = 0;
#endif

#ifdef USB_CMD_CONTROL
	//Force user to go into curb climbing mode
	if (usb_climb_state == 1){
	    button_prev_state = 1;
	}
#endif

#ifdef ONE_BUTTON_CONTROL_CURB_CLIMBING
	//when button3 is pressed, Extend climbing wheel until both wheel touches the ground
	if ((button_state.button3 == 1 || button_prev_state == 1) && climb_first_iteration == 1) {
	    button_prev_state = 1;
	    //Put both leg to same initial zero position for easier curb climbing mode detection
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

		/* 3 different scenerio to abort the climbing up task
		 1. The angle calculated is not feasible
		 2. The leg rotate more than it supposed to
		 3. The curb height is too low where climbing up is unnecessary
		 */
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
#if USB_CMD_CONTROL
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
#endif
	vTaskDelay(1000);
    }
}

void Task_Battery(void *param) {
    batteryHandler battery;

    Battery_Init(&battery, &huart2);
    uint8_t error_count = 0;
    uint16_t voltage_level = 25; //nominal voltage

    while (1) {
	//Data acquisition from BMS
	if (Battery_GetState(&battery) != HAL_OK) {
	    error_count++;
	}
	else{
	    voltage_level = battery.battery_info.total_voltage;
	    error_count = 0;
	}

	if(uxQueueSpacesAvailable(queue_battery_level) == 0){
	    //force the top message out
	    uint8_t dummy;
	    xQueueReceive(queue_battery_level, &dummy, 0);
	}
	xQueueSend(queue_battery_level, (void*)&voltage_level, 0);
	vTaskDelay(pdMS_TO_TICKS(10000));
	//Error Handling
	if(error_count > 50){
	    //Cannot read from BMS
	    uint8_t dummy;
	    (void)dummy;
	}
    }
}

/**@brief  Lights out LED based on differrent operation mode.
 * @param  mode 	Refer to Operation_Mode enum.
 * @retval None.
 */
static void LED_Mode_Configuration(Operation_Mode mode) {
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

/**@brief  Hub motor move forward  by preset dist.
 * @param  dist Distance to be moved by hub motor
 * @retval True if climb forward the specified distance.
 */
static bool climbingForward(float dist){
    static uint32_t prev_tick = 0;
    static int32_t prev_enc;
    static bool first_loop = true;
    static float dist_remaining;

    float rps = (dist >= 0) ? 1.0 : -1.0; //rad/s

    //Initialize static variable if first loop
    if (first_loop) {
	prev_enc = hub_encoder_feedback.encoder_2;
	prev_tick = HAL_GetTick();
	first_loop = false;
	dist_remaining = dist;
    }
    //As a fail safe to make sure the hub is functional
    //Sometime, the connector is loose and cause error in transmission
    if (xTaskGetTickCount() - last_hub_rx_t > 1500) {
	lifting_mode = DANGER;
	xTaskNotify(task_control, 0, eNoAction);
	return false;
    }

    //Calculate distance travelled and save the remaining distance to local static for next call
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

/**@brief  Control rear and back wheel to set encoder position using PID controller.
 * @param  enc 	 Position setpoint for the pid controller
 * @param  pid_t Controller for position
 * @retval True if motor has reached desired position.
 */
static bool goto_pos(int enc, PID_t pid_t) {
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
//	    speed[BACK_INDEX] = 0;
//	    if (fabs(speed[BACK_INDEX]) < 5)
//		pid_reset(backClimb_pid);
	    return false;
	}
    }
    return false;
}

/**@brief  Lifting process with desired front and back encoder position.
 * @param  front_enc 	 Position setpoint for the front pid controller
 * @param  back_enc	 Position setpoint for the back pid controller
 * @retval True if both motor has reached desired setpoint.
 */
static bool in_climb_process(int front_enc, int back_enc) {
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

    /*
     * 1. lift the front climbing wheel up until it reach it maximum pos
     * The process is controlled by PID on the front climbing wheel
     * the maximum pos is when the climbing wheel is below the wheelchair base
     */
    goto_pos(front_enc, frontClimb_pid);
    goto_pos(back_enc, backClimb_pid);

    if (fabs(speed[FRONT_INDEX]) >= 5 || fabs(speed[BACK_INDEX]) >= 5)
	is_lifting = true;
    else
	is_lifting = false;
    /*
     *2. During lifting, due to fixed point at the back climbing wheel.
     *The wheelchair would be pulled back if the back wheel not traveling while the its lifting
     *The Therefore, lifting of back wheel and hub motor need to work at the same time to make sure the wheelchair is not moving back.
     *The Pull back of wheelchair would cause the front climbing wheel to slip from the curb
     */
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
