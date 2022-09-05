/*
 * initializerthread.cpp
 *
 *  Created on: Sep 5, 2022
 *      Author: danie
 */


#include "initializerthread.hpp"

namespace threads{


void initializerThread(void* pvParameters){


	//system state variable
	enum System_State sys_state = INIT;

	//calibration flags
	bool imu_config_flag, imu_calib_flag = false;

	sensors::BNO055* localImuRef = ((initializerThreadArgs*)pvParameters)->pxIMU;
	actuators::BLHelis* localMotorsRef = ((initializerThreadArgs*)pvParameters)->pxMotors;
	communications::NRF24* localCommsRef = ((initializerThreadArgs*)pvParameters)->pxComms;
	SemaphoreHandle_t xInitializerMutex = *((initializerThreadArgs*)pvParameters)->pxInitializerMutex;

	xSemaphoreTake(xInitializerMutex, (TickType_t)0);

	//initializer thread is a state machine
	while(1){

		switch(sys_state){

			  	  case INIT:
			  	  	  {
			  	  		  imu_config_flag = localImuRef->configSensor();

			  	  		  if(imu_config_flag){
			  	  			  sys_state = IMU_CALIB_INIT;
			  	  		  }
			  	  	  }
			  		  break;

			  	  case IMU_CALIB_INIT:
			  	  	  {
					  	  //CODE FOR CALIBRATING THE SENSOR
			  		  	  imu_calib_flag = localImuRef->Read_IMU_Calib_Status();
			  		  	  bool test = localImuRef->Read_Calib_Params();

			  		  	  if(imu_calib_flag){
			  			  	  sys_state = IMU_CALIB_DONE;
			  		  	  }
			  	  	  }
			  		  break;



			  	  case IMU_CALIB_DONE:
			  	  	  {
			  		  	  //blink an LED, transistion to RTOS
					  	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
					  	  vTaskDelay(1000);
					  	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
					  	  vTaskDelay(1000);
					  	  sys_state = MOTOR_INIT;
			  	  	  }
			  		  break;

			  	  case MOTOR_INIT:
			  	  	  {
			  	  		  sys_state = MOTOR_INIT_DONE;
			  	  	  }
			  		  break;

			  	  case MOTOR_INIT_DONE:
			  	  	  {
			  		  	  sys_state = COMMS_INIT;
			  	  	  }
			  	  	  break;

			  	  case COMMS_INIT:
			  	  	  {
			  	  		  sys_state = COMMS_INIT_DONE;
			  	  	  }
			  	  	  break;


			  	  case COMMS_INIT_DONE:
			  	  	  {
			  	  		  sys_state = RTOS;
			  	  	  }
			  	  	  break;

			  	  case RTOS:
			  	  	  {

						  break;
			  	  	  }
			  	  	  break;


		}
	}

	//give back the initialization semaphore, unblocking the other threads
	xSemaphoreGive(&xInitializerMutex);
	vTaskDelete(NULL); //deletes self

}

}
