/*
 * mutexes.hpp
 *
 *  Created on: Sep 6, 2022
 *      Author: danie
 */

#ifndef SRC_THREADS_MUTEXES_HPP_
#define SRC_THREADS_MUTEXES_HPP_
#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"

SemaphoreHandle_t xSharedStateMutex;
SemaphoreHandle_t xSharedOutputMutex;
SemaphoreHandle_t xInitializerMutex;


#endif /* SRC_THREADS_MUTEXES_HPP_ */
