/*
 * battery.h
 *
 *  Created on: 30 Nov 2021
 *      Author: ray
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define NUM_BATTERY 14


typedef struct{
	uint16_t total_voltage; 			/*!< Unit:10mV >*/
	uint16_t current;					/*!< Unit:10mA, +ve:charging, -ve:discharging>*/
	uint16_t remaining_capacity;		/*!< Unit:10mAh >*/
	uint16_t nominal_capacity;			/*!< Unit:10mAh>*/
	uint16_t cycles;					/*!< Unit:cycle, charging cycle>*/
	uint16_t production_date;			/*!< Refer to datasheet >*/
	uint16_t balanced_state[2];			/*!< Refer to datasheet  >*/
	uint16_t protection_state;			/*!< 1: protection occured, 0: no, Note:Refer to datasheet to know what protection triggered>*/
	uint8_t software_version;			/*!< Refer to datasheet >*/
	uint8_t remaining_capacity_RSOC;	/*!< Unit:%, remaining capacity in percentage >*/
	uint8_t FET_control_status;			/*!< Read Bit0: Charging Bit1: Discharging, 0: Off, 1: On  >*/
	uint8_t battery_number;				/*!< No. of battery in series >*/
	uint8_t NTC_number;					/*!< No. of NTC >*/
	uint16_t *NTC_content;				/*!< Size: 2*NTC_Nnumber, Unit: 0.1K>*/
}batteryBasicInfo;

typedef struct{
	UART_HandleTypeDef* huart;
	batteryBasicInfo battery_info;
}batteryHandler;

/*
 * brief Initialize battery handler struct to all zero
 * param battery_handler 	pointer to battery handler
 * param  huart				pointer to uart handler that in charge in reading the battery
 */
void Battery_Init(batteryHandler* battery_handler, UART_HandleTypeDef* huart);

//TODO: Need to remove use of dynamic allocation. (BAD HABIT in embedded!)
//Can try to use circular buffer instead
/*
 * brief De-Initialize battery handler dynamic memory to zero
 * param battery_handler 	pointer to battery handler
 */
void Battery_DeInit(batteryHandler* battery_handler);

/*
 * brief Tell uart to transmit message via DMA to get the battery latest state
 * param battery_handler 	pointer to battery handler
 */
void Battery_GetState(batteryHandler* battery_handler);

/*
 * brief Process message that is received from the uart dma
 * param battery_handler 	pointer to battery handler
 */
void Battery_ReadState(batteryHandler* battery_handler, uint8_t receive_buf[]);



#endif /* INC_BATTERY_H_ */
