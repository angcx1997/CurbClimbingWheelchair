/*
 * MPUXX50.c
 *
 *  Created on: Feb 27, 2022
 *      Author: MarkSherstan
 */

#include "mpu6050.h"
#include "main.h"
#include "us_delay.h"

static void HAL_I2C_GPIO_Init(I2C_HandleTypeDef *hi2c, uint8_t state);

/// @brief Set the IMU address, check for connection, reset IMU, and set full range scale.
/// @param I2Cx Pointer to I2C structure config.
/// @param addr Hex address based on AD0 pin - 0x68 low or 0x69 high.
/// @param aScale Set accelerometer full scale range: 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
/// @param gScale Set gyroscope full scale range: 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
/// @param tau Set tau value for the complementary filter (typically 0.98).
/// @param dt Set sampling rate in seconds determined by the timer interrupt.
HAL_StatusTypeDef MPU_begin(I2C_HandleTypeDef *I2Cx, uint8_t addr, uint8_t aScale, uint8_t gScale, float tau, float dt)
{
    // Save values
    _addr = addr << 1;
    _tau = tau;
    _dt = dt;

    // Initialize variables
    uint8_t check;
    uint8_t select;

    // Confirm device
    HAL_I2C_Mem_Read(I2Cx, _addr, WHO_AM_I, 1, &check, 1, I2C_TIMOUT_MS);

    // TODO: If 9250 or 6050 fails could it trigger the opposite check???
    if ((check == WHO_AM_I_9250_ANS) || (check == WHO_AM_I_6050_ANS))
    {
        // Startup / reset the sensor
        select = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS);

        // Set the full scale ranges
        MPU_writeAccFullScaleRange(I2Cx, aScale);
        MPU_writeGyroFullScaleRange(I2Cx, gScale);

        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

/// @brief Set the accelerometer full scale range.
/// @param I2Cx Pointer to I2C structure config.
/// @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
void MPU_writeAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale)
{
    // Variable init
    uint8_t select;

    // Set the value
    switch (aScale)
    {
    case AFSR_2G:
        aScaleFactor = 16384.0;
        select = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_4G:
        aScaleFactor = 8192.0;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_8G:
        aScaleFactor = 4096.0;
        select = 0x10;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_16G:
        aScaleFactor = 2048.0;
        select = 0x18;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        aScaleFactor = 8192.0;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
}

/// @brief Set the gyroscope full scale range.
/// @param I2Cx Pointer to I2C structure config.
/// @param gScale Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
void MPU_writeGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale)
{
    // Variable init
    uint8_t select;

    // Set the value
    switch (gScale)
    {
    case GFSR_250DPS:
        gScaleFactor = 131.0;
        select = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_500DPS:
        gScaleFactor = 65.5;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_1000DPS:
        gScaleFactor = 32.8;
        select = 0x10;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_2000DPS:
        gScaleFactor = 16.4;
        select = 0x18;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        gScaleFactor = 65.5;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
}

/// @brief Read raw data from IMU.
/// @param I2Cx Pointer to I2C structure config.
HAL_StatusTypeDef MPU_readRawData(I2C_HandleTypeDef *I2Cx)
{
    HAL_StatusTypeDef status;
    // Init buffer
    uint8_t buf[14];

    // Subroutine for reading the raw data
    status = HAL_I2C_Mem_Read(I2Cx, _addr, ACCEL_XOUT_H, 1, buf, 14, I2C_TIMOUT_MS);

    // Bit shift the data
    rawData.ax = buf[0] << 8 | buf[1];
    rawData.ay = buf[2] << 8 | buf[3];
    rawData.az = buf[4] << 8 | buf[5];
    // temperature = buf[6] << 8 | buf[7];
    rawData.gx = buf[8] << 8 | buf[9];
    rawData.gy = buf[10] << 8 | buf[11];
    rawData.gz = buf[12] << 8 | buf[13];

    return status;
}

/// @brief Find offsets for each axis of gyroscope.
/// @param I2Cx Pointer to I2C structure config.
/// @param numCalPoints Number of data points to average.
void MPU_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints)
{
    // Init
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    // Zero guard
    if (numCalPoints == 0)
    {
        numCalPoints = 1;
    }

    // Save specified number of points
    for (uint16_t ii = 0; ii < numCalPoints; ii++)
    {
        MPU_readRawData(I2Cx);
        x += rawData.gx;
        y += rawData.gy;
        z += rawData.gz;
        HAL_Delay(3);
    }

    // Average the saved data points to find the gyroscope offset
    gyroCal.x = (float)x / (float)numCalPoints;
    gyroCal.y = (float)y / (float)numCalPoints;
    gyroCal.z = (float)z / (float)numCalPoints;
}

/// @brief Calculate the real world sensor values.
/// @param I2Cx Pointer to I2C structure config.
HAL_StatusTypeDef MPU_readProcessedData(I2C_HandleTypeDef *I2Cx)
{
    HAL_StatusTypeDef status;
    // Get raw values from the IMU
    status = MPU_readRawData(I2Cx);

    // Convert accelerometer values to g's
    sensorData.ax = rawData.ax / aScaleFactor;
    sensorData.ay = rawData.ay / aScaleFactor;
    sensorData.az = rawData.az / aScaleFactor;

    // Compensate for gyro offset
    sensorData.gx = rawData.gx - gyroCal.x;
    sensorData.gy = rawData.gy - gyroCal.y;
    sensorData.gz = rawData.gz - gyroCal.z;

    // Convert gyro values to deg/s
    sensorData.gx /= gScaleFactor;
    sensorData.gy /= gScaleFactor;
    sensorData.gz /= gScaleFactor;

    return status;
}

/// @brief Calculate the attitude of the sensor in degrees using a complementary filter.
/// @param I2Cx Pointer to I2C structure config.
HAL_StatusTypeDef MPU_calcAttitude(I2C_HandleTypeDef *I2Cx)
{
    HAL_StatusTypeDef status;

    // Read processed data
    status = MPU_readProcessedData(I2Cx);

    // Complementary filter
    float accelPitch = atan2(sensorData.ay, sensorData.az) * RAD2DEG;
    float accelRoll = atan2(sensorData.ax, sensorData.az) * RAD2DEG;

    attitude.r = _tau * (attitude.r - sensorData.gy * _dt) + (1 - _tau) * accelRoll;
    attitude.p = _tau * (attitude.p + sensorData.gx * _dt) + (1 - _tau) * accelPitch;
    attitude.y += sensorData.gz * _dt;

    return status;
}

HAL_StatusTypeDef MPU6050_I2C_Reset(I2C_HandleTypeDef *I2Cx)
{
    uint8_t SDA_state;
    uint8_t SCL_state;

    //STOP I2C peripheral
    HAL_I2C_MspDeInit(I2Cx);

    //Initialize both as input with pullup to check I2C state
    HAL_I2C_GPIO_Init(I2Cx, 0);

    SCL_state = HAL_GPIO_ReadPin(IMU_I2C1_SCL_GPIO_Port, IMU_I2C1_SCL_Pin);
    //If SCL is off during idle, hardware reset is a must
    if (SCL_state == GPIO_PIN_RESET)
	return HAL_ERROR;

    uint8_t clockCount = 20; // > 2x9 clock
    SDA_state = HAL_GPIO_ReadPin(IMU_I2C1_SDA_GPIO_Port, IMU_I2C1_SDA_Pin);
    while(SDA_state == GPIO_PIN_RESET && (clockCount > 0)){
	clockCount--;
	//Initialize SCL as output port to spam clock pulse
	HAL_I2C_GPIO_Init(I2Cx, 1);
	HAL_GPIO_WritePin(IMU_I2C1_SCL_GPIO_Port, IMU_I2C1_SCL_Pin, GPIO_PIN_RESET);
	delay_us(10);
	HAL_GPIO_WritePin(IMU_I2C1_SCL_GPIO_Port, IMU_I2C1_SCL_Pin, GPIO_PIN_SET);
	delay_us(10);


	//Initialize both as input with pullup to check I2C state
	HAL_I2C_GPIO_Init(I2Cx, 0);
	SDA_state = HAL_GPIO_ReadPin(IMU_I2C1_SDA_GPIO_Port, IMU_I2C1_SDA_Pin);
	SCL_state = HAL_GPIO_ReadPin(IMU_I2C1_SCL_GPIO_Port, IMU_I2C1_SCL_Pin);

	if (SCL_state == GPIO_PIN_RESET)
	    return HAL_ERROR;
	if (SDA_state == GPIO_PIN_SET)
	    return HAL_OK;
    }
    if (SDA_state == GPIO_PIN_RESET){
	//Simulate repeated start and stop condition
	HAL_I2C_GPIO_Init(I2Cx, 2);
	HAL_GPIO_WritePin(IMU_I2C1_SDA_GPIO_Port, IMU_I2C1_SDA_Pin, GPIO_PIN_SET);
	delay_us(10);

	HAL_GPIO_WritePin(IMU_I2C1_SDA_GPIO_Port, IMU_I2C1_SDA_Pin, GPIO_PIN_RESET);
	delay_us(10);

	HAL_GPIO_WritePin(IMU_I2C1_SDA_GPIO_Port, IMU_I2C1_SDA_Pin, GPIO_PIN_SET);
	delay_us(10);


	//Initialize both as input with pullup to check I2C state
	HAL_I2C_GPIO_Init(I2Cx, 0);
	SDA_state = HAL_GPIO_ReadPin(IMU_I2C1_SDA_GPIO_Port, IMU_I2C1_SDA_Pin);
	SCL_state = HAL_GPIO_ReadPin(IMU_I2C1_SCL_GPIO_Port, IMU_I2C1_SCL_Pin);

	if (SCL_state == GPIO_PIN_RESET)
	    return HAL_ERROR;
	if (SDA_state == GPIO_PIN_SET)
	    return HAL_OK;
    }
    HAL_I2C_MspInit(I2Cx);
    return HAL_OK;
}

void HAL_I2C_GPIO_Init(I2C_HandleTypeDef *hi2c, uint8_t state) {
    if (hi2c->Instance == I2C1 && state == 0) {
	GPIO_InitTypeDef GPIO_InitStruct = {
		0
	};
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**I2C1 GPIO Configuration
	 PB8     ------> I2C1_SCL
	 PB9     ------> I2C1_SDA
	 */
	HAL_GPIO_WritePin(GPIOB, IMU_I2C1_SCL_Pin | IMU_I2C1_SDA_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = IMU_I2C1_SCL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = IMU_I2C1_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Peripheral clock enable */
	/* USER CODE BEGIN I2C1_MspInit 1 */

	/* USER CODE END I2C1_MspInit 1 */
    }
    else if (hi2c->Instance == I2C1 && state == 1) {
    	GPIO_InitTypeDef GPIO_InitStruct = {
    		0
    	};
    	__HAL_RCC_GPIOB_CLK_ENABLE();
    	/**I2C1 GPIO Configuration
    	 PB8     ------> I2C1_SCL
    	 PB9     ------> I2C1_SDA
    	 */
    	HAL_GPIO_WritePin(GPIOB, IMU_I2C1_SCL_Pin | IMU_I2C1_SDA_Pin, GPIO_PIN_RESET);

    	GPIO_InitStruct.Pin = IMU_I2C1_SCL_Pin;
    	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    	GPIO_InitStruct.Pull = GPIO_PULLUP;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    	GPIO_InitStruct.Pin = IMU_I2C1_SDA_Pin;
    	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    	GPIO_InitStruct.Pull = GPIO_PULLUP;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    	/* Peripheral clock enable */
    	/* USER CODE BEGIN I2C1_MspInit 1 */

    	/* USER CODE END I2C1_MspInit 1 */
        }
    else if (hi2c->Instance == I2C1 && state == 2) {
       	GPIO_InitTypeDef GPIO_InitStruct = {
       		0
       	};
       	__HAL_RCC_GPIOB_CLK_ENABLE();
       	/**I2C1 GPIO Configuration
       	 PB8     ------> I2C1_SCL
       	 PB9     ------> I2C1_SDA
       	 */
       	HAL_GPIO_WritePin(GPIOB, IMU_I2C1_SCL_Pin | IMU_I2C1_SDA_Pin, GPIO_PIN_RESET);

       	GPIO_InitStruct.Pin = IMU_I2C1_SCL_Pin;
       	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
       	GPIO_InitStruct.Pull = GPIO_PULLUP;
       	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

       	GPIO_InitStruct.Pin = IMU_I2C1_SDA_Pin;
       	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
       	GPIO_InitStruct.Pull = GPIO_PULLUP;
       	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

       	/* Peripheral clock enable */
       	/* USER CODE BEGIN I2C1_MspInit 1 */

       	/* USER CODE END I2C1_MspInit 1 */
           }
}
