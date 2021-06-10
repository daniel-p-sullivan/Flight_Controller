/*
 * actuators.h
 *
 *  Created on: Jun 9, 2021
 *      Author: danie
 */

#ifndef SRC_ACTUATORS_H_
#define SRC_ACTUATORS_H_

#include "main.h"
#include "stm32f4xx_hal.h"

void Init_Motor_PWM(void);
void Set_Motor_PWM(uint16_t* compare1, uint16_t* compare2, uint16_t* compare3, uint16_t* compare4);

#endif /* SRC_ACTUATORS_H_ */
