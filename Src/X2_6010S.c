/*
 * X2_6010S.c
 *
 *  Created on: May 22, 2021
 *      Author: angcx
 */

#include "X2_6010S.h"
extern uint8_t receive_buf[15];

typedef enum {
	MOTOR_ENABLE = 0x00,
	MOTOR_BRAKE = 0x01,
	MOTOR_DISABLE = 0x02,
	MOTOR_CLR_ALARM = 0x03
} Motor_Command;

enum MOTOR_ADDRESS_ENUM {
	MOTOR_1 = 0xA1,
	MOTOR_2 = 0xA2,
	MOTOR_3 = 0xA3,
	MOTOR_4 = 0xA4
};

typedef enum {
	MOTOR_ENCODER_FEEDBACK = 0x80,
	MOTOR_CONTROLLER_FEEDBACK = 0x82,
	MOTOR_VELOCITY_CURRENT_FEEDBACK = 0x83,
} Motor_Feedback_Command;

void HubMotor_Init() {
	HAL_GPIO_WritePin(HubM_IO_SON_GPIO_Port, HubM_IO_SON_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HubM_IO_NOT_GPIO_Port, HubM_IO_NOT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HubM_IO_POT_GPIO_Port, HubM_IO_POT_Pin, GPIO_PIN_RESET);
}

HAL_StatusTypeDef HubMotor_SendCommand(float m1_ang_speed, float m2_ang_speed) {
	int16_t motor1_speed, motor2_speed;
	HAL_StatusTypeDef status;
	//convert velocity into pulse/second
	motor1_speed = -(int16_t) (m1_ang_speed * 4096.0 / (2 * M_PI));
	motor2_speed = (int16_t) (m2_ang_speed * 4096.0 / (2 * M_PI));

	uint8_t send_buf[15];
	send_buf[0] = 0xAA;
	send_buf[1] = 0xA4;
	send_buf[2] = 0x0E;
	send_buf[3] = 0x00;
	send_buf[4] = MOTOR_ENABLE;
	send_buf[5] = MOTOR_ENCODER_FEEDBACK;

	//Set acceleration to constant by default
	//time taken from 0 to 1000rpm
	uint16_t acceleration = 200;
	uint8_t msb_acce = (uint8_t) ((acceleration & 0xFF00) >> 8);
	uint8_t lsb_acce = (uint8_t) (acceleration & 0x00FF);
	send_buf[6] = lsb_acce;
	send_buf[7] = msb_acce;

	//Set maximum torque
	//Value: 0 - 450 (300 by default)
	uint16_t max_torque = 350;
	uint8_t msb_max_torque = (uint8_t) ((max_torque & 0xFF00) >> 8);
	uint8_t lsb_max_torque = (uint8_t) (max_torque & 0x00FF);
	send_buf[8] = lsb_max_torque;
	send_buf[9] = msb_max_torque;

	//Set motor1 speed
	uint8_t msb_motor1_speed = (uint8_t) ((motor1_speed & 0xFF00) >> 8);
	uint8_t lsb_motor1_speed = (uint8_t) (motor1_speed & 0x00FF);
	send_buf[10] = lsb_motor1_speed;
	send_buf[11] = msb_motor1_speed;

	//Set motor2 speed
	uint8_t msb_motor2_speed = (uint8_t) ((motor2_speed & 0xFF00) >> 8);
	uint8_t lsb_motor2_speed = (uint8_t) (motor2_speed & 0x00FF);
	send_buf[12] = lsb_motor2_speed;
	send_buf[13] = msb_motor2_speed;

	//checksum byte
	uint16_t sum = (uint16_t) send_buf[0] + (uint16_t) send_buf[1] + (uint16_t) send_buf[2] + (uint16_t) send_buf[3]
			+ (uint16_t) send_buf[4] + (uint16_t) send_buf[5] + (uint16_t) send_buf[6] + (uint16_t) send_buf[7]
			+ (uint16_t) send_buf[8] + (uint16_t) send_buf[9] + (uint16_t) send_buf[10] + (uint16_t) send_buf[11]
			+ (uint16_t) send_buf[12] + (uint16_t) send_buf[13];

	send_buf[14] = (uint8_t) (sum & 0x00FF);

	status = HAL_UART_Transmit(&huart3, send_buf, 15, 10);
	if (status != HAL_OK)
		return status;

	status = HAL_UART_Receive_DMA(&huart3, receive_buf, 15);
	if (status != HAL_OK)
		return status;
	return HAL_OK;
}

bool HubMotor_CalculateChecksum(uint8_t receive_buf[]) {
	uint16_t sum = (uint16_t) receive_buf[0] + (uint16_t) receive_buf[1] + (uint16_t) receive_buf[2]
			+ (uint16_t) receive_buf[3] + (uint16_t) receive_buf[4] + (uint16_t) receive_buf[5]
			+ (uint16_t) receive_buf[6] + (uint16_t) receive_buf[7] + (uint16_t) receive_buf[8]
			+ (uint16_t) receive_buf[9] + (uint16_t) receive_buf[10] + (uint16_t) receive_buf[11]
			+ (uint16_t) receive_buf[12] + (uint16_t) receive_buf[13];
	return ((sum & 0xFF) == receive_buf[14]) && (receive_buf[0] == 0xAA) && (receive_buf[1] == 0xA4) && (receive_buf[3] == 0x00) && (receive_buf[4] == 0x00);
}

Encoder_Feedback HubMotor_ReceiveCallback(uint8_t receive_buf[])
{
	Encoder_Feedback encoder_feedback;
	encoder_feedback.encoder_1 = (receive_buf[9] << 24) + (receive_buf[8] << 16) + (receive_buf[7] << 8)
			+ (receive_buf[6]);
	encoder_feedback.encoder_2 = (receive_buf[13] << 24) + (receive_buf[12] << 16)
			+ (receive_buf[11] << 8) + (receive_buf[10]);
	return encoder_feedback;
}

