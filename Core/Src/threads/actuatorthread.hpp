/*
 * actuatorthread.hpp
 *
 *  Created on: Sep 1, 2022
 *      Author: danie
 */

#ifndef SRC_THREADS_ACTUATORTHREAD_HPP_
#define SRC_THREADS_ACTUATORTHREAD_HPP_


#include "../actuators/actuators.hpp"
#include "../state/state.hpp"
#include "../actuators/blhelis.hpp"
#include "cmsis_os.h"

namespace threads{

struct actuatorThreadArgs{
	state::QuadControlActions* output;
	actuators::BLHelis* motors;
	SemaphoreHandle_t* pxSharedOutputMutex;
};


void actuatorThread(void* pvParameters);


}


#endif /* SRC_THREADS_ACTUATORTHREAD_HPP_ */
