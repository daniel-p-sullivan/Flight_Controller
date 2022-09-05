/*
 * sensorthread.hpp
 *
 *  Created on: Sep 1, 2022
 *      Author: danie
 */

#ifndef SRC_THREADS_SENSORTHREAD_HPP_
#define SRC_THREADS_SENSORTHREAD_HPP_

#include "../state/state.hpp"
#include "../sensors/sensors.hpp"
#include "../sensors/bno055.hpp"
#include "cmsis_os.h"
#include "FreeRTOS.h"


namespace threads{


struct sensorThreadArgs{
	state::QuadStateVector* state;
	sensors::BNO055* imu;
	SemaphoreHandle_t* pxSharedStateMutex;
	SemaphoreHandle_t* pxInitializerMutex;
};


void sensorThread(void* pvParameters);



}

#endif /* SRC_THREADS_SENSORTHREAD_HPP_ */
