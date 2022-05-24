/*
 * MPUXX50.h
 *
 *  Created on: Feb 27, 2022
 *      Author: MarkSherstan
 */

#ifndef MPUXX50_H_
#define MPUXX50_H_

// Libs
#include <stdint.h>
#include <math.h>
#include <stm32f4xx.h>
#include <stdbool.h>

// Constants
#define RAD2DEG 57.2957795131

// Defines
#define WHO_AM_I_6050_ANS 0x68
#define WHO_AM_I_9250_ANS 0x71
#define WHO_AM_I          0x75
#define AD0_LOW           0x68
#define AD0_HIGH          0x69
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define PWR_MGMT_1        0x6B
#define ACCEL_XOUT_H      0x3B
#define I2C_TIMOUT_MS     5

// Full scale ranges
enum gyroscopeFullScaleRange
{
    GFSR_250DPS,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accelerometerFullScaleRange
{
    AFSR_2G,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

// Structures
struct RawData
{
    int16_t ax, ay, az, gx, gy, gz;
} rawData;

struct SensorData
{
    float ax, ay, az, gx, gy, gz;
} sensorData;

struct GyroCal
{
    float x, y, z;
} gyroCal;

struct Attitude
{
    float r, p, y;
} attitude;

// Variables
uint8_t _addr;
float _dt, _tau;
float aScaleFactor, gScaleFactor;

// Functions
HAL_StatusTypeDef MPU_begin(I2C_HandleTypeDef *I2Cx, uint8_t addr, uint8_t aScale, uint8_t gScale, float tau, float dt);
void MPU_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints);
HAL_StatusTypeDef MPU_calcAttitude(I2C_HandleTypeDef *I2Cx);
HAL_StatusTypeDef MPU_readRawData(I2C_HandleTypeDef *I2Cx);
HAL_StatusTypeDef MPU_readProcessedData(I2C_HandleTypeDef *I2Cx);
void MPU_writeGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale);
void MPU_writeAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale);

/**
 * Used to reset mpu6050 with mcu connection
 * Sometime, with multiple reset, mpu6050 sdl being stretch low.
 * Therefore, reset the SDA and SCL line
 * @param I2Cx i2c handler
 * @return HAL_StatusTypeDef HAL_OK if mpu6050 able to work fine
 */
HAL_StatusTypeDef MPU6050_I2C_Reset(I2C_HandleTypeDef *I2Cx);

#endif /* MPUXX50_H_ */
