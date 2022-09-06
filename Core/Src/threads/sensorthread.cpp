/*
 * sensorthread.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */


#include "sensorthread.hpp"
#include "FreeRTOS.h"

namespace threads{


void sensorThread(void* pvParameters){

	state::QuadStateVector localState;
	state::QuadStateVector* globalStateRef = ((sensorThreadArgs*)pvParameters)->state;
	sensors::BNO055* imuRef = ((sensorThreadArgs*)pvParameters)->imu;
	SemaphoreHandle_t xSharedStateMutex = *(((sensorThreadArgs*)pvParameters)->pxSharedStateMutex);
	SemaphoreHandle_t xInitializerMutex = *(((sensorThreadArgs*)pvParameters)->pxInitializerMutex);

	const TickType_t xFrequency = 1000; //scheduler is running at 1Khz, this thread will be able to run at that freq too
	TickType_t xLastWakeTime;

	auto retvar = xSemaphoreTake(xInitializerMutex, (TickType_t)1000); //needs to get this mutex to continue exec
															  //cannot grab this until initialization is done
	xSemaphoreGive(xInitializerMutex); //proceed into inf loop now that initialization is done


	while(1){

		xLastWakeTime = xTaskGetTickCount();
		vTaskDelayUntil(&xLastWakeTime, xFrequency); //blocks


		localState = imuRef->readIMU();

		xSemaphoreTake(xSharedStateMutex, (TickType_t) 0);
		*globalStateRef = localState;
		xSemaphoreGive(xSharedStateMutex);

	}

}








}
