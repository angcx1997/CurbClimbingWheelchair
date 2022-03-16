/*
 * utilities.h
 *
 *  Created on: Mar 15, 2022
 *      Author: ray
 */

#ifndef INC_UTILITIES_H_
#define INC_UTILITIES_H_

#include "stdint.h"
#include "stdbool.h"

/*
 * @brief calculate 16bit checksum
 * @param *arr 	pointer to array
 * @param size 	size of array that wish to calculate for checksum
 * 		If array of 8, wish to calculate checksum,
 * 		and last 2 byte use to store checksum,
 * 		input (8-2=)6 as size.
 * @retval 16bit checksum
 */
uint16_t CalculateChecksum_16bit(uint8_t *arr, uint16_t size);

/*
 * @brief calculate 8bit checksum
 * @param *arr 	pointer to array
 * @param size 	size of array that wish to calculate for checksum
 * 		If array of 8, wish to calculate checksum,
 * 		and last 2 byte use to store checksum,
 * 		input (8-2=)6 as size.
 * @retval 8bit checksum
 */
uint8_t CalculateChecksum_8bit(uint8_t *arr, uint16_t size);

/*
 * @brief validate 16bit checksum, if 16bit, last 2 byte of array,
 * 		will automatically use as value for checksum
 * @param *arr 	pointer to array
 * @param size 	size of array
 * @retval bool true if checksum is correct
 */
bool ValidateChecksum_16bit(uint8_t *arr, uint16_t size);

/*
 * @brief validate 8bit checksum, if 8bit, last 1 byte of array,
 * 		will automatically use as value for checksum
 * @param *arr 	pointer to array
 * @param size 	size of array
 * @retval bool true if checksum is correct
 */
bool ValidateChecksum_8bit(uint8_t *arr, uint16_t size);


#endif /* INC_UTILITIES_H_ */
