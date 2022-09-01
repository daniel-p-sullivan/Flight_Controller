/*
 * sensorthread.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */


#include "../sensors/sensors.hpp"
#include "../state/state.hpp"
#include "../sensors/bno055.hpp"

namespace threads{


void sensorThread(state::QuadStateVector& state, sensors::BNO055 imu){

	state::QuadStateVector localState;


	while(1){

		localState = imu.readIMU();

		//lock(state)
		state = localState;
		//unlock(state)

	}

}








}
