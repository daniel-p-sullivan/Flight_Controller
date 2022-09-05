/*
 * actuatorthread.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */


#include "actuatorthread.hpp"

namespace threads{


void actuatorThread(void* pvParameters){


	state::QuadControlActions* globalOutputRef = ((actuatorThreadArgs*)pvParameters)->output;
	actuators::BLHelis* motorsRef = ((actuatorThreadArgs*)pvParameters)->motors;
	state::QuadControlActions localOutput;
	SemaphoreHandle_t xSharedOutputMutex = *(((actuatorThreadArgs*)pvParameters)->pxSharedOutputMutex);
	SemaphoreHandle_t xInitializerMutex = *((actuatorThreadArgs*)pvParameters)->pxInitializerMutex;

	const TickType_t xFrequency = 1000; //scheduler is running at 1Khz, this thread will be able to run at that freq too
	TickType_t xLastWakeTime;

	xSemaphoreTake(xInitializerMutex, (TickType_t)0); //needs to get this mutex to continue exec
													  //cannot grab this until initialization is done
	xSemaphoreGive(xInitializerMutex); //proceed into inf loop now that initialization is done

	while(1){

		xLastWakeTime = xTaskGetTickCount();
		vTaskDelayUntil(&xLastWakeTime, xFrequency); //blocks

		//get measurement
		xSemaphoreTake(xSharedOutputMutex, (TickType_t)0);
		localOutput = *globalOutputRef; //copy the output into local var then release
		xSemaphoreGive(xSharedOutputMutex);

		motorsRef->actuateMotors(localOutput);
	}

}



}
