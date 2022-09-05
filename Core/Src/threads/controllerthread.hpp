/*
 * controllerthread.hpp
 *
 *  Created on: Sep 1, 2022
 *      Author: danie
 */

#ifndef SRC_THREADS_CONTROLLERTHREAD_HPP_
#define SRC_THREADS_CONTROLLERTHREAD_HPP_

#include "../controllers/controllers.hpp"
#include "../state/state.hpp"
#include "cmsis_os.h"

namespace threads{

struct controllerThreadArgs{
	state::QuadStateVector* state;
	state::QuadControlActions* output;
	SemaphoreHandle_t* pxSharedStateMutex;
	SemaphoreHandle_t* pxSharedOutputMutex;
	SemaphoreHandle_t* pxInitializerMutex;
};

void controllerThread(void* pvParameters);

}


#endif /* SRC_THREADS_CONTROLLERTHREAD_HPP_ */
