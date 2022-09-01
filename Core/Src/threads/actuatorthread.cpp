/*
 * actuatorthread.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */


#include "actuatorthread.hpp"

namespace threads{


void actuatorThread(void* pvParameters){

	state::QuadControlActions& globalOutputRef = ((actuatorThreadArgs*)pvParameters)->output;
	actuators::BLHelis motorsRef = ((actuatorThreadArgs*)pvParameters)->motors;
	state::QuadControlActions localOutput;
	SemaphoreHandle_t xSharedOutputMutex = *(((actuatorThreadArgs*)pvParameters)->pxSharedOutputMutex);


	while(1){

		//get measurement
		xSemaphoreTake(xSharedOutputMutex, (TickType_t)0);
		localOutput = globalOutputRef; //copy the output into local var then release
		xSemaphoreGive(xSharedOutputMutex);

		motorsRef.actuateMotors(localOutput);
	}

}



}
