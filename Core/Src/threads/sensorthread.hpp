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
#include "./mutexes.hpp"
#include "cmsis_os.h"
#include "FreeRTOS.h"

extern SemaphoreHandle_t xSharedStateMutex;
extern SemaphoreHandle_t xInitializerMutex;

namespace threads{


struct sensorThreadArgs{
	state::QuadStateVector* state;
	I2C_HandleTypeDef i2c;
	SemaphoreHandle_t* pxSharedStateMutex;
	SemaphoreHandle_t* pxInitializerMutex;
};


void sensorThread(void* pvParameters);



}

#endif /* SRC_THREADS_SENSORTHREAD_HPP_ */
