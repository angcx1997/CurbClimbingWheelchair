/*
 * joystick.h
 * Careful with using this file
 * Only one joystick is allowed to use
 * If wish to use more than one,
 * please allocate the static variable prev_joystick into joystick_hanlder struct
 *  Created on: 26 Nov 2021
 *      Author: ray
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

typedef struct
{
  int  	x;
  int  	y;
}JoystickHandle;

/*
 * brief Calculate joystick position
 * param joystick_handler 	pointer to joystick_handler
 */
void Joystick_CalculatePos(JoystickHandle* joystick_handler);

#endif /* INC_JOYSTICK_H_ */
