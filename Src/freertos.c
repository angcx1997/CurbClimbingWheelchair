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
/* USER CODE BEGIN Includes */
#include <adc.h>
#include <battery.h>
#include <bd25l.h>
#include <briter_encoder_rs485.h>
#include <button.h>
#include <differentialDrive.h>
#include <DifferentialDrivetoSabertooth.h>
#include <encoder.h>
#include <encoder_util.h>
#include <FreeRTOS.h>
#include <joystick.h>
#include <main.h>
#include <math.h>
#include <mpu6050.h>
#include <portmacro.h>
#include <projdefs.h>
#include <PID.h>
#include <PID_base.h>
#include <queue.h>
#include <semphr.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_can.h>
#include <stm32f4xx_hal_def.h>
#include <stm32f4xx_hal_dma.h>
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_i2c.h>
#include <stm32f4xx_hal_spi.h>
#include <stm32f4xx_hal_tim.h>
#include <stm32f4xx_hal_uart.h>
#include <string.h>
#include <sys/_stdint.h>
#include <Sabertooth.h>
#include <task.h>
#include <tfmini.h>
#include <timers.h>
#include <usb_device.h>
#include <X2_6010S.h>

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

typedef enum {
    ERROR_WHEEL_ENCODER = 0x00,
    ERROR_CURB_DETECTOR,
    ERROR_CLIMB_ENCODER,
    ERROR_IMU,
    ERROR_HUB_MOTOR

} sensor_fault_e;

typedef enum{
    DOCKING_STAGE_NONE = 0,
    DOCKING_STAGE_1 = 1,
    DOCKING_STAGE_2 = 2,
    DOCKING_STAGE_3 = 3,
    DOCKING_STAGE_4 = 4,
    DOCKING_STAGE_COMPLETE = 5
}Docking_State_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef ULONG_MAX
#define ULONG_MAX 0xFFFFFFFF
#endif
//#define DEBUGGING
#define DATA_xxLOGGING
#ifndef MAX
    #define MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif
#ifndef MIN
    #define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif
#define TO_RAD(x) ((x) * 0.01745329251)
#define TO_DEG(x) ((x) * 57.2957795131)

#define BITSET(byte,nbit)   ((byte) |=  (1<<(nbit)))
#define BITCLEAR(byte,nbit) ((byte) &= ~(1<<(nbit)))
#define BITCHECK(byte,nbit) ((byte) & 1<<(nbit))

#define SIGN(x) (((x)>=0)?1:-1)
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
extern TaskHandle_t task_normalDrive;
extern TaskHandle_t task_joystick;
extern TaskHandle_t task_climbing;
extern TaskHandle_t task_usb;
extern TaskHandle_t task_climb_encoder;
extern TaskHandle_t task_imu;
extern TaskHandle_t task_wheel_encoder;
extern TaskHandle_t task_switches;

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
const Gear_Level gear_level = GEAR2; //change the speed level if need higher speed
speedConfig speed_config = {
	.max_vel = (float)gear_level/100,	//gear level/100
	.min_vel = -(float)gear_level/ 100 /2 , //Need to be lower than |max_vel| to make sure reverse is slower and safer
	.min_acc = - (float)gear_level / 100,	//max_vel*2
	.max_acc = (float)gear_level/ 100 / 4
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
uint32_t back_encoder_input = 0;
uint8_t finish_climbing_flag = 0; //1 if climbing motion finish

uint8_t usb_climb_state = 0;

//Base wheel control
Sabertooth_Handler sabertooth_handler;
extern Briter_Encoder_t base_encoder[2];
Debug_Tick_t encoder_tick_taken = {
	0
};

extern wheel_velocity_t base_velocity[2];

//IMU
volatile float heading_angle = 0;

#ifdef DATA_LOGGING
//Declare variable
//System ID logging
//    volatile uint16_byte_u LOG_battery;
//    volatile float_byte_u LOG_left_pwm;
//    volatile float_byte_u LOG_right_pwm;
//    volatile uint32_byte_u LOG_left_enc;
//    volatile uint32_byte_u LOG_right_enc;
//    volatile float_byte_u LOG_left_enc_vel;
//    volatile float_byte_u LOG_right_enc_vel;
//PID Tracking Logging
volatile float_byte_u LOG_left_ref_vel;
volatile float_byte_u LOG_right_ref_vel;
volatile float_byte_u LOG_left_enc_vel;
volatile float_byte_u LOG_right_enc_vel;
volatile uint32_byte_u LOG_tick;
//    volatile uint32_t tick_count = 0;
DataLogger_Msg_t datalog_msg;
uint8_t usbBuffer[64];
float v = 0.5;
#endif
volatile uint32_t tick_count = 0;

//Battery level
uint32_t battery_level = 25;

//Speed PID Controller for 2 wheel
PID_Struct base_pid[2];
const float base_kp[2] = {
	0.51588, 0.51588
}; /*!< Proportional constant in both left and right wheel PID */
const float base_ki[2] = {
	103.1758, 103.9472
}; /*!< Integral constant in both left and right wheel PID */
const float kd[2] = {
	0.0, 0.0
}; /*!< Derivative constant in both left and right wheel PID */
const float kf[2] = {
	12.48577, 12.51787
}; /*!< Feedforward constant in both left and right wheel PID */
float pwm_volt[2]; /*!< PWM voltage to be input into motor */
float reference_vel[2]; /*!< Setpoint for PID controller */
float max_i_output = 20; /*!< To clamp integral output */
float pid_freq = 1000;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void LED_Mode_Configuration(Operation_Mode mode);
static bool climbingForward(float dist); //return true if in the process of moving forward
static bool goto_pos(int enc, PID_t pid_t); //return true if still in the process of reaching the position
static bool in_climb_process(int front_enc, int back_enc);
static void base_velocity_controller(float left_vel, float right_vel, PID_Struct *pid_left, PID_Struct *pid_right);
static bool move_forward(float dist_desired);
static bool turn_angle(float angle_desired, float curr_angle);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Task_Control(void *param) {
    /*
     * Pre-empt all other task and immediately stop running wheel
     */
    uint32_t ulNotifiedValue;
    while (1) {
	xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
	if (lifting_mode == DANGER) {
	    portENTER_CRITICAL();
	    vTaskSuspendAll();
	    runMotor(&rearMotor, 0);
	    runMotor(&backMotor, 0);
	    HubMotor_SendCommand(0, 0);
	    MotorStop(&sabertooth_handler);
	    LED_Mode_Configuration(DANGER);
	    portEXIT_CRITICAL();
	}
    }
}

void Task_NormalDrive(void *param) {
    /*
     * Mainly control wheelchair base wheel, task run when normal driving
     */
    differentialDrive_Handler differential_drive_handler;

    //declare a struct to hold linear and angular velocity
    struct Command_Velocity {
	float linear;
	float angular;
    };
    struct Command_Velocity cmd_vel = {
	    0, 0
    };

    //Initialize base wheel
    int motor_output_1 = 0;
    int motor_output_2 = 0;
    memset(&base_velocity[LEFT_INDEX], 0, sizeof(wheel_velocity_t));
    memset(&base_velocity[RIGHT_INDEX], 0, sizeof(wheel_velocity_t));
    DDrive_Init(&differential_drive_handler, FREQUENCY);
    MotorInit(&sabertooth_handler, 128, &huart6);
    MotorStartup(&sabertooth_handler);
    MotorStop(&sabertooth_handler);

    //Battery level
    uint16_t voltage_level = 25;

    //Setup right wheel PID
    PID_Init(&base_pid[RIGHT_INDEX]);
    PID_setPIDF(&base_pid[RIGHT_INDEX], base_kp[RIGHT_INDEX], base_ki[RIGHT_INDEX], kd[RIGHT_INDEX], kf[RIGHT_INDEX]);
    PID_setOutputLimits(&base_pid[RIGHT_INDEX], -max_i_output, max_i_output);
    PID_setMaxIOutput(&base_pid[RIGHT_INDEX], max_i_output);
    PID_setMinIOutput(&base_pid[RIGHT_INDEX], -max_i_output);
    PID_setFrequency(&base_pid[RIGHT_INDEX], pid_freq);

    //Setup left wheel PID
    PID_Init(&base_pid[LEFT_INDEX]);
    PID_setPIDF(&base_pid[LEFT_INDEX], base_kp[LEFT_INDEX], base_ki[LEFT_INDEX], kd[LEFT_INDEX], kf[LEFT_INDEX]);
    PID_setOutputLimits(&base_pid[LEFT_INDEX], -max_i_output, max_i_output);
    PID_setMaxIOutput(&base_pid[LEFT_INDEX], max_i_output);
    PID_setMinIOutput(&base_pid[LEFT_INDEX], -max_i_output);
    PID_setFrequency(&base_pid[LEFT_INDEX], pid_freq);

    int moveForwardFlag = false;
    while (1) {
	if (lifting_mode == NORMAL) {
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
	if (uxQueueMessagesWaiting(queue_battery_level)) {
	    xQueueReceive(queue_battery_level, &voltage_level, 0);
	    voltage_level /= 100;
	}
	else {
	    voltage_level = 25;
	}


	DDrive_SpeedMapping(&differential_drive_handler, cmd_vel.angular, cmd_vel.linear, gear_level);
	dDriveToST_Adapter(&differential_drive_handler, &sabertooth_handler);
//	base_velocity_controller(differential_drive_handler.m_leftMotor, differential_drive_handler.m_rightMotor, &base_pid[LEFT_INDEX], &base_pid[RIGHT_INDEX]);

	//Docking task
	uint8_t docking_stage = 0;
	if(BITCHECK(docking_stage, DOCKING_STAGE_COMPLETE)){
	    base_velocity_controller(0, 0, &base_pid[LEFT_INDEX], &base_pid[RIGHT_INDEX]);
	    BITCLEAR(docking_stage, DOCKING_STAGE_COMPLETE);
	}
	else if(BITCHECK(docking_stage, DOCKING_STAGE_1)){
	    if (move_forward(1)){
		BITSET(docking_stage,DOCKING_STAGE_2);
		BITSET(docking_stage, DOCKING_STAGE_COMPLETE);
	    }
	}
	else if(BITCHECK(docking_stage, DOCKING_STAGE_2)){
	    if (turn_angle(45, 0)){
		BITSET(docking_stage,DOCKING_STAGE_3);
		BITSET(docking_stage, DOCKING_STAGE_COMPLETE);
	    }
	}
	else if(BITCHECK(docking_stage, DOCKING_STAGE_3)){
	    if (move_forward(1)){
		BITSET(docking_stage,DOCKING_STAGE_NONE);
		BITSET(docking_stage, DOCKING_STAGE_COMPLETE);
	    }
	}
//	if (moveForwardFlag == false) {
//	    moveForwardFlag = turn_angle(45, heading_angle);
////	    moveForwardFlag = move_forward(1);
//	}
//	if (moveForwardFlag == true) {
//	    base_velocity_controller(0, 0, &base_pid[LEFT_INDEX], &base_pid[RIGHT_INDEX]);
//	    attitude.y = 0;
//	}

#ifdef DATA_xxxLOGGING
	LOG_left_ref_vel.data = v;
		LOG_right_ref_vel.data = v;
		LOG_left_enc_vel.data = base_velocity[LEFT_INDEX].velocity;
		LOG_right_enc_vel.data = base_velocity[RIGHT_INDEX].velocity;
	/*============================================================================*/
	/*Input wave*/
	uint16_t sampling_rate = 750;
	tick_count++;
	//generate sine wave
//	float v = 0.75 * sin(2 * M_PI * tick_count/sampling_rate);
	//Fourier wave
//	v = 0.3* (sin(2 * M_PI * 0.1 * tick_count/sampling_rate) + sin(2 * M_PI * 0.2 * tick_count/sampling_rate)
//		    + sin(2 * M_PI * 0.4 * tick_count/sampling_rate) + sin(2 * M_PI * tick_count/sampling_rate));
	//Square wave
//	if (tick_count % 300 == 0)
//	    v = -v;

//	int motor_output_1 = v * SABERTOOTH_MAX_ALLOWABLE_VALUE;
//	int motor_output_2 = v * SABERTOOTH_MAX_ALLOWABLE_VALUE;
//	MotorThrottle(&sabertooth_handler, TARGET_2, motor_output_1);
//	MotorThrottle(&sabertooth_handler, TARGET_1, motor_output_2);
	/*============================================================================*/
	/*Joystick input*/
    	DDrive_SpeedMapping(&differential_drive_handler, cmd_vel.angular, cmd_vel.linear, gear_level);
    	dDriveToST_Adapter(&differential_drive_handler, &sabertooth_handler);

    	//Define variable
//    	LOG_left_pwm.data = v;
//    	LOG_right_pwm.data = v;
    	LOG_left_pwm.data = differential_drive_handler.m_leftMotor;
	LOG_right_pwm.data = differential_drive_handler.m_rightMotor;
#endif
	vTaskDelay(5);
    }
}

void Task_Wheel_Encoder(void *param) {
    //Initialize encoder sensor for base wheel
    BRITER_RS485_Init(&(base_encoder[0]), 0x02, &huart4);
    BRITER_RS485_Init(&(base_encoder[1]), 0x01, &huart4);
    //Flag to alternate the address of rs485 tx to avoid congestion
    uint8_t base_encoder_tx_flag = 0;
    uint32_t irq_retval;

    //Filter for velocity
    const float velocity_filter = 0.85;
    float prev_velocity[2] = {
	    0
    };

    TickType_t tick = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(5); //execution period

    while (1) {
	/*Data Acquisition*/
	HAL_StatusTypeDef briter_dma_status;
	if (base_encoder_tx_flag % 2 == 0) {
	    briter_dma_status = BRITER_RS485_GetValue_DMA_TX(&base_encoder[0]);
//	    base_encoder_tx_flag++;
	}
	else {
	    briter_dma_status = BRITER_RS485_GetValue_DMA_TX(&base_encoder[1]);
//	    base_encoder_tx_flag--;
	}
	if (briter_dma_status == HAL_OK)
	    BRITER_RS485_GetValue_DMA_RX(base_encoder);

	if (xTaskNotifyWait(0x00, ULONG_MAX, &irq_retval, pdMS_TO_TICKS(5)) == pdTRUE) {
	    if (irq_retval != BRITER_RS485_ERROR) {
		if (base_encoder_tx_flag % 2 == 0) {
		    base_encoder[LEFT_INDEX].encoder_value = irq_retval;
		    calculateVelocity(&base_velocity[LEFT_INDEX], base_encoder[LEFT_INDEX].encoder_value);
		    //Data smoothening
		    base_velocity[LEFT_INDEX].velocity = base_velocity[LEFT_INDEX].velocity * velocity_filter
			    + (1 - velocity_filter) * prev_velocity[LEFT_INDEX];
		    prev_velocity[LEFT_INDEX] = base_velocity[LEFT_INDEX].velocity;
		    last_rs485_enc_t[LEFT_INDEX] = xTaskGetTickCount();
		    base_encoder_tx_flag++;
		}
		else {
		    base_encoder[RIGHT_INDEX].encoder_value = irq_retval;
		    calculateVelocity(&base_velocity[RIGHT_INDEX], base_encoder[RIGHT_INDEX].encoder_value);
		    last_rs485_enc_t[RIGHT_INDEX] = xTaskGetTickCount();
		    //Data smoothening
		    base_velocity[RIGHT_INDEX].velocity = base_velocity[RIGHT_INDEX].velocity * velocity_filter
			    + (1 - velocity_filter) * prev_velocity[RIGHT_INDEX];
		    prev_velocity[RIGHT_INDEX] = base_velocity[RIGHT_INDEX].velocity;
		    base_encoder_tx_flag--;
		}
	    }
	}
	else {
	    continue;
	}
	/*Error Handling*/
	//If encoder is faulty, no data reception, suspend all task
	if ((xTaskGetTickCount() - last_rs485_enc_t[0]) > 1000 || (xTaskGetTickCount() - last_rs485_enc_t[1]) > 1000) {
	    lifting_mode = DANGER;
	    xTaskNotify(task_control, ERROR_WHEEL_ENCODER, eSetValueWithOverwrite);
	}
	vTaskDelayUntil(&tick, period);
    }
}

void Task_Curb_Detector(void *param) {
    while (1) {
	//Distance sensor data acquisition is managed by DMA
	HAL_UART_Receive_DMA(&huart1, tf_rx_buf, TFMINI_RX_SIZE);
	//Error Handling
	//UART error cause by noise flag, framing, overrun error happens during multibuffer dma communication
	if (xTaskGetTickCount() - last_tf_mini_t > 500) {
	    lifting_mode = DANGER;
	    xTaskNotify(task_control, ERROR_CURB_DETECTOR, eSetValueWithOverwrite);
	}
	vTaskDelay(5);
    }
}

void Task_Climb_Encoder(void *param) {
    //Initialize climbing encoder sensor and start front distance sensor data reception
    ENCODER_Init();

    //Only start all operation after sensor are turned on.
    xTaskNotify(task_climbing, 0, eNoAction);

    while (1) {
	/*Climbing Sensor Data Acquisition and Processing*/
	/*Data Acquisition*/
	//Read encoder data
	ENCODER_Get_Angle(&encoderBack);
	ENCODER_Get_Angle(&encoderFront);

	/*Error Handling*/
	//If encoder is faulty, no data reception, suspend all task
	if ((xTaskGetTickCount() - last_can_rx_t[0]) > 1000 || (xTaskGetTickCount() - last_can_rx_t[1]) > 1000) {
	    lifting_mode = DANGER;
	    xTaskNotify(task_control, ERROR_CLIMB_ENCODER, eSetValueWithOverwrite);
	}
	vTaskDelay(10);
    }
}

void Task_Switches(void *param) {
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

    TickType_t tick = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100); //execution period
    while (1) {
	//Button user interface
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

	vTaskDelayUntil(&tick, period);
    }
}

void Task_IMU(void *param) {
    uint32_t mpu_error_count = 0;
    uint32_t state_count = xTaskGetTickCount();

    const float filter = 0.98;
    const float sampling_rate = 0.01;
    const uint8_t mpu6050_addr = WHO_AM_I_6050_ANS;

    //Initialize MPU6050, Reset the SDA line if cannot transmit message
    while (MPU_begin(&hi2c1, mpu6050_addr, AFSR_2G, GFSR_250DPS, filter, sampling_rate) != HAL_OK) {
	if (xTaskGetTickCount() - state_count > 50)
	    if (MPU6050_I2C_Reset(&hi2c1) == HAL_ERROR) {
		lifting_mode = DANGER;
		xTaskNotify(task_control, ERROR_IMU, eSetValueWithOverwrite);
	    }
    }

    MPU_calibrateGyro(&hi2c1, 100);

    while (1) {
	//Sensor acquisition of IMU
	if (MPU_calcAttitude(&hi2c1) == HAL_OK) {
	    mpu_error_count = 0;
	    heading_angle = attitude.y;
	}
	else {
	    mpu_error_count++;
	}

	if (mpu_error_count > 10) {
	    MPU6050_I2C_Reset(&hi2c1);
	    MPU_begin(&hi2c1, mpu6050_addr, AFSR_2G, GFSR_250DPS, filter, sampling_rate);
	}
	vTaskDelay(10);
    }
}

/**
 * Joystick interface
 * For acquisition of joystick data
 */
void Task_Joystick(void *param) {

    //Warning: Period cannot be shorter than 100
    const TickType_t period = pdMS_TO_TICKS(100); //execution period

    JoystickHandle joystick_handler;

    //Initialize AD7606
    ADC_Init();

    while (1) {
	ADC_DataRequest();
	//Use mutex to safeguard joystick data being accessed and modified by task_normal_drive
	if (xSemaphoreTake(mutex_joystick, pdMS_TO_TICKS(5)) == pdTRUE) {
	    if (xTaskNotifyWait(0x00, ULONG_MAX, 0, 50) == pdTRUE) {
		int16_t adc_rawData[8];
		ADC_Read(adc_rawData);
		joystick_handler.x = adc_rawData[2];
		joystick_handler.y = adc_rawData[1];
		Joystick_CalculatePos(&joystick_handler);
	    }
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

	vTaskDelay(period);
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

    xTaskNotifyWait(0x00, ULONG_MAX, (uint32_t*) 0, portMAX_DELAY);
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
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
#ifdef DATA_LOGGING
    DataLogger_Init(&datalog_msg);
#endif
    while (1) {
#ifdef DATA_LOGGING
	//Define variable
//    	LOG_left_enc.data = base_encoder[LEFT_INDEX].encoder_value;
//    	LOG_right_enc.data = base_encoder[RIGHT_INDEX].encoder_value;
//    	LOG_left_enc_vel.data = base_velocity[LEFT_INDEX].velocity;
//    	LOG_right_enc_vel.data = base_velocity[RIGHT_INDEX].velocity;

	//Debugging purpose
//	LOG_left_pwm.data = 0.54321;
//	LOG_right_pwm.data = 0.56789;
//	LOG_left_enc.data = 0x12345670;
//	LOG_right_enc.data = 0x89ABCDEF;
//	LOG_left_enc_vel.data = 0.12345;
//	LOG_right_enc_vel.data = 0.67890;
//
//	DataLogger_AddMessage(&datalog_msg, LOG_battery.array, sizeof(LOG_battery.array));
//	DataLogger_AddMessage(&datalog_msg, LOG_left_pwm.array, sizeof(LOG_left_pwm.array));
//	DataLogger_AddMessage(&datalog_msg, LOG_right_pwm.array, sizeof(LOG_right_pwm.array));
//	DataLogger_AddMessage(&datalog_msg, LOG_left_enc.array, sizeof(LOG_left_enc.array));
//	DataLogger_AddMessage(&datalog_msg, LOG_right_enc.array, sizeof(LOG_right_enc.array));
//	DataLogger_AddMessage(&datalog_msg, LOG_left_enc_vel.array, sizeof(LOG_left_enc_vel.array));
//	DataLogger_AddMessage(&datalog_msg, LOG_right_enc_vel.array, sizeof(LOG_right_enc_vel.array));

	LOG_tick.data = xTaskGetTickCount();
	DataLogger_AddMessage(&datalog_msg, LOG_left_ref_vel.array, sizeof(LOG_left_ref_vel.array));
	DataLogger_AddMessage(&datalog_msg, LOG_right_ref_vel.array, sizeof(LOG_right_ref_vel.array));
	DataLogger_AddMessage(&datalog_msg, LOG_left_enc_vel.array, sizeof(LOG_left_enc_vel.array));
	DataLogger_AddMessage(&datalog_msg, LOG_right_enc_vel.array, sizeof(LOG_right_enc_vel.array));
	DataLogger_AddMessage(&datalog_msg, LOG_tick.array, sizeof(LOG_tick.array));
	//4. After added all the message, call complete tx message to add check to ensure data integrity
	DataLogger_CompleteTxMessage(&datalog_msg);
	CDC_Transmit_FS(datalog_msg.pMsg, datalog_msg.size);
#endif
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
	vTaskDelay(5);
    }
}

void Task_Battery(void *param) {
    batteryHandler battery;

    Battery_Init(&battery, &huart2);
    uint8_t error_count = 0;
    uint16_t voltage_level = 2500; //nominal voltage

    while (1) {
	//Data acquisition from BMS
	if (Battery_GetState(&battery) != HAL_OK) {
	    error_count++;
	}
	else {
	    voltage_level = battery.battery_info.total_voltage;
	    error_count = 0;
	}

	if (uxQueueSpacesAvailable(queue_battery_level) == 0) {
	    //force the top message out
	    uint8_t dummy;
	    xQueueReceive(queue_battery_level, &dummy, 0);
	}
	xQueueSend(queue_battery_level, (void* )&voltage_level, 0);
	battery_level = voltage_level / 100;
	vTaskDelay(pdMS_TO_TICKS(10000));
	//Error Handling
	if (error_count > 50) {
	    //Cannot read from BMS
	    uint8_t dummy;
	    (void) dummy;
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
static bool climbingForward(float dist) {
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
	xTaskNotify(task_control, ERROR_HUB_MOTOR, eSetValueWithOverwrite);
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

/**
 * @brief  Base motor move forward  by preset dist.
 * @param  dist Distance to be moved by base wheel
 * @retval True if move forward the specified distance.
 */

static bool move_forward(float dist_desired) {
    static int32_t prev_enc[2];
    static bool first_loop = true;
    static float dist_travelled;
    //PID setup
    static PID_t moveforward_pid = NULL;
    static struct pid_controller moveforward_ctrl;
    static float moveforward_input = 0, moveforward_output = 0, moveforward_setpoint = 0;
    float moveforward_kp = 0.4, moveforward_ki = 0.25, moveforward_kd = 0.000;

    //A scaling factor to account for distance correction (slip, friction, inaccurate wheel diameter measurement, etc)
    //0.97 is obtained through trial and error by running wheelchair with setpoint of 1m.
    dist_desired *= 0.97;

    if (moveforward_pid == NULL) {
	moveforward_pid = pid_create(&moveforward_ctrl, &moveforward_input, &moveforward_output, &moveforward_setpoint,
		moveforward_kp, moveforward_ki, moveforward_kd);
	pid_limits(moveforward_pid, -0.15, 0.15);
	pid_sample(moveforward_pid, 1);
	pid_auto(moveforward_pid);
    }

    //Initialize static variable if first loop
    if (first_loop) {
	prev_enc[LEFT_INDEX] = base_velocity[LEFT_INDEX].total_position;
	prev_enc[RIGHT_INDEX] = base_velocity[RIGHT_INDEX].total_position;
	first_loop = false;
	dist_travelled = 0;
	return false;
    }

    //Check whether pid need to be computed or the dist is within tolerable range
    if (!first_loop && fabs(dist_desired - dist_travelled) > 0.001) {
	//Use raw encoder value to calculate distance travelled, instead of using velocity
	//To minimize sensor noise and increase accuracy
	//Individual Distance travelled = change in encoder tick / PPR * wheel circumference
	//Total distance travel at t = (Distance by left + Distance by right) / 2
	int32_t left_distance_travelled = base_velocity[LEFT_INDEX].total_position - prev_enc[LEFT_INDEX];
	int32_t right_distance_travelled = base_velocity[RIGHT_INDEX].total_position - prev_enc[RIGHT_INDEX];
	float tmp = (M_PI * WHEEL_DIA) / (2 * BRITER_RS485_PPR);
	float dist_travel_t = tmp * (float) (left_distance_travelled + right_distance_travelled);
	dist_travelled += dist_travel_t;
	//Refresh PID controller
	moveforward_input = dist_travelled;
	moveforward_setpoint = dist_desired;
	pid_compute(moveforward_pid);
	//Output wheel velocity
	base_velocity_controller(moveforward_output, moveforward_output, &base_pid[LEFT_INDEX], &base_pid[RIGHT_INDEX]);
	prev_enc[LEFT_INDEX] = base_velocity[LEFT_INDEX].total_position;
	prev_enc[RIGHT_INDEX] = base_velocity[RIGHT_INDEX].total_position;
	return false;
    }
    else {
	first_loop = true;
	return true;
    }
}

/**
 * @brief Base motor turn on spot by preset angle in degree.
 * @param angle_desired (in degree)target angle to be rotated[-180,180]
 * @param curr_angle 	(in degree)latest angle when function called[-180,180]
 * @return boolean 	True if finish turning specified angle
 */
float v_turn = 0;
//float turnangle_input = 0, turnangle_output = 0, turnangle_setpoint = 0;
static bool turn_angle(float angle_desired, float curr_angle) {
    static bool first_loop = true;
    static PID_t turnangle_pid = NULL;
    static float init_heading = 0;
    static float angle_travelled = 0;
    static int32_t prev_enc[2];
    //PID setup
    static struct pid_controller turnangle_ctrl;
    static float turnangle_input = 0, turnangle_output = 0, turnangle_setpoint = 0;
    float turnangle_kp = 0.05, turnangle_ki = 0.035, turnangle_kd = 0.00;

    curr_angle = TO_RAD(curr_angle);
    angle_desired = TO_RAD(angle_desired) * 0.97;

    if (turnangle_pid == NULL) {
	turnangle_pid = pid_create(&turnangle_ctrl, &turnangle_input, &turnangle_output, &turnangle_setpoint,
		turnangle_kp, turnangle_ki, turnangle_kd);
	pid_limits(turnangle_pid, -0.1, 0.1);
	pid_sample(turnangle_pid, 1);
	pid_auto(turnangle_pid);
    }

#ifdef IMU_TURN_ANGLE
    //Initialize static variable if first loop
    if (first_loop) {
	first_loop = false;
	float x = cosf(curr_angle);
	float y = sinf(curr_angle);
	init_heading = atan2f(y, x);
	x = cosf(angle_desired);
	y = sinf(angle_desired);
	turnangle_setpoint = atan2f(y, x);
	return false;
    }

    float x = cosf(curr_angle);
    float y = sinf(curr_angle);
    turnangle_input = atan2f(y, x) - init_heading;
    x = cosf(turnangle_input);
    y = sinf(turnangle_input);
    turnangle_input = atan2f(y, x);

    //Check whether pid need to be computed or the dist is within tolerable range
    if (!first_loop && fabs(angle_desired - turnangle_input) > 0.01) {
	//Use IMU to trace angle turn
	//Refresh PID controller
//	float x = cosf(curr_angle);
//	float y = sinf(curr_angle);
//	turnangle_/	float x = cosf(angle_travelled);
	//	float y = sinf(angle_travelled);
	//	turnangle_ininput = atan2f(y, x) - init_heading;
//	x = cosf(turnangle_input);
//	y = sinf(turnangle_input);
//	turnangle_input = atan2f(y, x);
	pid_compute(turnangle_pid);
	//Output wheel velocity
	//Convert angular velocity to individual wheel speed
	//Formula:
	//V_r = w * L / 2R
	//V_l = - V_r
	float v = turnangle_output * BASE_WIDTH / WHEEL_DIA;
	v_turn = turnangle_output * BASE_WIDTH / WHEEL_DIA;
	base_velocity_controller(-v, v, &base_pid[LEFT_INDEX], &base_pid[RIGHT_INDEX]);
	return false;
    }
    else {
	first_loop = true;
	return true;
    }
#else
    //Initialize static variable if first loop
    if (first_loop) {
	first_loop = false;
	angle_travelled = 0;
	prev_enc[LEFT_INDEX] = base_velocity[LEFT_INDEX].total_position;
	prev_enc[RIGHT_INDEX] = base_velocity[RIGHT_INDEX].total_position;
	return false;
    }

    //Check whether pid need to be computed or the dist is within tolerable range
    if (!first_loop && fabs(angle_desired - angle_travelled) > 0.01) {
	//Use wheel odometry to trace angle turn
	int32_t left_enc_tick = base_velocity[LEFT_INDEX].total_position - prev_enc[LEFT_INDEX];
	int32_t right_enc_tick= base_velocity[RIGHT_INDEX].total_position - prev_enc[RIGHT_INDEX];
	float tmp = (M_PI * WHEEL_DIA) / (BASE_WIDTH * BRITER_RS485_PPR);
	//Add angle travelled at t to total angle travelled
	angle_travelled += tmp * (float)(right_enc_tick - left_enc_tick);
	turnangle_input = angle_travelled;
	turnangle_setpoint = angle_desired;
	//Refresh PID controller
	pid_compute(turnangle_pid);
	//Output wheel velocity
	//Convert angular velocity to individual wheel speed
	//Formula:
	//V_r = w * L / 2R
	//V_l = - V_r
	float v = turnangle_output * BASE_WIDTH / WHEEL_DIA;
	v_turn = turnangle_output * BASE_WIDTH / WHEEL_DIA;
	base_velocity_controller(-v, v, &base_pid[LEFT_INDEX], &base_pid[RIGHT_INDEX]);
	prev_enc[LEFT_INDEX] = base_velocity[LEFT_INDEX].total_position;
	prev_enc[RIGHT_INDEX] = base_velocity[RIGHT_INDEX].total_position;
	return false;
    }
    else {
	first_loop = true;
	return true;
    }
#endif
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

/**
 * Use for velocity PID control loop
 * @param left_vel 	left reference velocity
 * @param right_vel	right reference velocity
 * @param pid_left 	left pid struct
 * @param pid_right	right pid struct
 */
static void base_velocity_controller(float left_vel, float right_vel, PID_Struct *pid_left, PID_Struct *pid_right) {
    float pwm_volt[2] = {
	    0
    };
    int motor_output[2] = {
	    0
    };
    if (fabs(left_vel) == 0 && fabs(base_velocity[LEFT_INDEX].velocity) < 0.05) {
	//Reset PID if it is too small
	pwm_volt[LEFT_INDEX] = 0;
	PID_reset(&base_pid[LEFT_INDEX]);
    }
    else {
	pwm_volt[LEFT_INDEX] = PID_getOutput(&base_pid[LEFT_INDEX], base_velocity[LEFT_INDEX].velocity, left_vel);
    }

    if (fabs(right_vel) == 0 && fabs(base_velocity[RIGHT_INDEX].velocity) < 0.05) {
	pwm_volt[RIGHT_INDEX] = 0;
	PID_reset(&base_pid[RIGHT_INDEX]);
    }
    else {
	pwm_volt[RIGHT_INDEX] = PID_getOutput(&base_pid[RIGHT_INDEX], base_velocity[RIGHT_INDEX].velocity, right_vel);
    }

    //TODO: Add situation to account for motor deadband
    const float deadband_vel[2] = {
	    0.15, 0.15
    };
    float deadband_volt[2] = {
	    0
    };
    deadband_volt[LEFT_INDEX] = deadband_vel[LEFT_INDEX] * kf[LEFT_INDEX];
    deadband_volt[RIGHT_INDEX] = deadband_vel[RIGHT_INDEX] * kf[RIGHT_INDEX];

    if (fabs(pwm_volt[LEFT_INDEX]) > 0.05 && fabs(pwm_volt[LEFT_INDEX]) < deadband_volt[LEFT_INDEX]) {
	pwm_volt[LEFT_INDEX] = deadband_vel[LEFT_INDEX] * SIGN(pwm_volt[LEFT_INDEX]);
    }
    if (fabs(pwm_volt[RIGHT_INDEX]) > 0.05 && fabs(pwm_volt[RIGHT_INDEX]) < deadband_volt[RIGHT_INDEX]) {
	pwm_volt[RIGHT_INDEX] = deadband_vel[RIGHT_INDEX] * SIGN(pwm_volt[RIGHT_INDEX]);
    }

    motor_output[LEFT_INDEX] = (float) (pwm_volt[LEFT_INDEX] / battery_level) * SABERTOOTH_MAX_ALLOWABLE_VALUE;
    motor_output[RIGHT_INDEX] = (float) (pwm_volt[RIGHT_INDEX] / battery_level) * SABERTOOTH_MAX_ALLOWABLE_VALUE;
    MotorThrottle(&sabertooth_handler, TARGET_2, motor_output[LEFT_INDEX]);
    MotorThrottle(&sabertooth_handler, TARGET_1, motor_output[RIGHT_INDEX]);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
