/*
 * actuatorthread.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */

#include "../actuators/actuators.hpp"
#include "../state/state.hpp"


namespace threads{

void actuatorThread(state::QuadControlActions& output, TIM_HandleTypeDef tim, actuators::BLHelis motors
){


	state::QuadControlActions myOutput;

	while(1){

		//get measurement
		//lock(output)
		myOutput = output; //copy the output into local var then release
		//unlock(output)

		motors.acuateMotors(myOutput);
	}

}



}
