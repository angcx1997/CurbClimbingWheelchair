/*
 * DifferentialDrivetoSabertooth.h
 *
 *  Created on: 9 Dec 2021
 *      Author: ray
 */

#ifndef INC_DIFFERENTIALDRIVETOSABERTOOTH_H_
#define INC_DIFFERENTIALDRIVETOSABERTOOTH_H_

#include "Sabertooth.h"
#include "differentialDrive.h"

/*
 * brief Convert from differential drive velocity to sabertooth suitable input
 * param dd_handler 	pointer to differentialDrive_Handler
 * param  st_handler	pointer to sabertooth handler
 */
void dDriveToST_Adapter(differentialDrive_Handler* dd_handler, Sabertooth_Handler *st_handler);

#endif /* INC_DIFFERENTIALDRIVETOSABERTOOTH_H_ */
