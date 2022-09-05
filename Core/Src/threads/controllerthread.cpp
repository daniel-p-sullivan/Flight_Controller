/*
 * controllerthread.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */


#include "controllerthread.hpp"

namespace threads{

void controllerThread(void* pvParameters){


	control::PI thrustController = control::PI(0, 0.1, 10, 2);
	control::PI yawController = control::PI(0, 0.1, 10, 2);
	control::PI rollController = control::PI(0, 0.1, 10, 2);
	control::PI pitchController = control::PI(0, 0.1, 10, 2);
	state::QuadStateVector localState;
	state::QuadControlActions localOutput;
	state::QuadStateVector* globalStateRef = ((controllerThreadArgs*)pvParameters)->state;
	state::QuadControlActions* globalOutputRef = ((controllerThreadArgs*)pvParameters)->output;
	SemaphoreHandle_t xSharedStateMutex = *(((controllerThreadArgs*)pvParameters)->pxSharedStateMutex);
	SemaphoreHandle_t xSharedOutputMutex = *(((controllerThreadArgs*)pvParameters)->pxSharedOutputMutex);
	SemaphoreHandle_t xInitializerMutex = *(((controllerThreadArgs*)pvParameters)->pxInitializerMutex);
	const TickType_t xFrequency = 1000;
	TickType_t xLastWakeTime;


	xSemaphoreTake(xInitializerMutex, (TickType_t)0); //needs to get this mutex to continue exec
														  //cannot grab this until initialization is done
	xSemaphoreGive(xInitializerMutex); //proceed into inf loop now that initialization is done

	while(1){
		xLastWakeTime = xTaskGetTickCount();
		vTaskDelayUntil(&xLastWakeTime, xFrequency); //blocks than instantly returns

		xSemaphoreTake(xSharedStateMutex, (TickType_t) 0); //nonblocking
		localState = *globalStateRef;
		xSemaphoreGive(xSharedStateMutex);


		localOutput.u1 = thrustController.calcOutput(localState.z);
		localOutput.u2 = rollController.calcOutput(localState.psi);
		localOutput.u3 = pitchController.calcOutput(localState.theta);
		localOutput.u4 = yawController.calcOutput(localState.phi);

		xSemaphoreTake(xSharedOutputMutex, (TickType_t) 0);
		*globalOutputRef = localOutput;
		xSemaphoreGive(xSharedOutputMutex);



	}


}

}
