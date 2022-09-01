/*
 * sensorthread.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */


#include "sensorthread.hpp"

namespace threads{


void sensorThread(void* pvParameters){

	state::QuadStateVector localState;
	state::QuadStateVector& globalStateRef = ((sensorThreadArgs*)pvParameters)->state;
	sensors::BNO055 imuRef = ((sensorThreadArgs*)pvParameters)->imu;
	SemaphoreHandle_t xSharedStateMutex = *(((sensorThreadArgs*)pvParameters)->pxSharedStateMutex);

	while(1){

		localState = imuRef.readIMU();

		xSemaphoreTake(xSharedStateMutex, (TickType_t) 0);
		globalStateRef = localState;
		xSemaphoreGive(xSharedStateMutex);

	}

}








}
