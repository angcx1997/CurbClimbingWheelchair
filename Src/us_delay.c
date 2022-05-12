/*
 * us_delay.c
 *
 *  Created on: 12 May 2022
 *      Author: ray
 */


#include "us_delay.h"

TIM_HandleTypeDef *us_tim;

void delay_us_init(TIM_HandleTypeDef* htim)
{
//    TIM_MasterConfigTypeDef sMasterConfig = {
//	    0
//    };
//
//    htim7.Instance = TIM7;
//    htim7.Init.Prescaler = 72;
//    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
//    htim7.Init.Period = 0xffff - 1;
//    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
//	Error_Handler();
//    }
//    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK) {
//	Error_Handler();
//    }

    us_tim = htim;
    HAL_TIM_Base_Start(us_tim);
}

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(us_tim,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(us_tim) < us);  // wait for the counter to reach the us input in the parameter
}
