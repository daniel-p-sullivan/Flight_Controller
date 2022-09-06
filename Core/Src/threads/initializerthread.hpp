/*
 * initializerthread.hpp
 *
 *  Created on: Sep 5, 2022
 *      Author: danie
 */

#ifndef SRC_THREADS_INITIALIZERTHREAD_HPP_
#define SRC_THREADS_INITIALIZERTHREAD_HPP_


#include "../actuators/actuators.hpp"
#include "../state/state.hpp"
#include "../sensors/sensors.hpp"
#include "../comms/communications.hpp"
#include "../actuators/blhelis.hpp"
#include "../sensors/bno055.hpp"
#include "../comms/nrf24.hpp"
#include "cmsis_os.h"
#include "FreeRTOS.h"


namespace threads{

enum System_State {
	INIT,
	IMU_CALIB_INIT,
	IMU_CALIB_DONE,
	MOTOR_INIT,
	MOTOR_INIT_DONE,
	COMMS_INIT,
	COMMS_INIT_DONE,
	RTOS
};

struct initializerThreadArgs{
	sensors::BNO055* pxIMU;
	actuators::BLHelis* pxMotors;
	communications::NRF24* pxComms;
	SemaphoreHandle_t* pxInitializerMutex;
	TaskHandle_t* pxInitializerThreadHandle;
};


void initializerThread(void* pvParameters);


}






#endif /* SRC_THREADS_INITIALIZERTHREAD_HPP_ */
