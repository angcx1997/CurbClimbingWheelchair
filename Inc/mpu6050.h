/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>
#include "stm32f4xx.h"


// MPU6050 structure
typedef struct {

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    float Ax;
    float Error_Ax;
    float Az;
    float Error_Az;
    float Ay;
    float Error_Ay;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float Gx;
    float Error_Gx;
    float Gy;
    float Error_Gy;
    float Gz;
    float Error_Gz;

    float Temperature;

    float KalmanAngleX;
    float KalmanAngleY;

    int count;
} MPU6050_t;


// Kalman structure
typedef struct {
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
} Kalman_t;


HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Calculate_Error(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt);

