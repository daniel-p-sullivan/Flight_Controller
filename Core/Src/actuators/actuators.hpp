/*
 * actuators.h
 *
 *  Created on: Jun 9, 2021
 *      Author: danie
 */

#ifndef SRC_ACTUATORS_HPP_
#define SRC_ACTUATORS_HPP_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "../state/state.hpp"

//drivers

namespace actuators{

typedef struct motor_sp{
	uint16_t m1_sp;
	uint16_t m2_sp;
	uint16_t m3_sp;
	uint16_t m4_sp;
} motor_sp;

class Motors{
public:
	void actuateMotors(state::QuadControlActions& ac);
};
}


#endif /* SRC_ACTUATORS_HPP_ */
