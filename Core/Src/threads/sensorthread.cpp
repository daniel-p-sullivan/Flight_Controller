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
	state::QuadStateVector* globalStateRef = ((sensorThreadArgs*)pvParameters)->state;
	I2C_HandleTypeDef& refI2c = ((sensorThreadArgs*)pvParameters)->i2c;
	sensors::BNO055 imu(refI2c);
	//SemaphoreHandle_t xSharedStateMutex = *(((sensorThreadArgs*)pvParameters)->pxSharedStateMutex);
	//SemaphoreHandle_t xInitializerMutex = *(((sensorThreadArgs*)pvParameters)->pxInitializerMutex);

	const TickType_t xFrequency = 1000; //scheduler is running at 1Khz, this thread will be able to run at that freq too
	TickType_t xLastWakeTime;

	auto retvar = xSemaphoreTake(xInitializerMutex, (TickType_t)0); //needs to get this mutex to continue exec
															  //cannot grab this until initialization is done
	bool imu_config_flag = false; bool imu_calib_flag = false;
	int state = 0;
	while(1){

			switch(state){
			case 0:
			{
				imu_config_flag = imu.configSensor();

				if(imu_config_flag){
					state = 1;
				}

			}break;
			case 1:
			{
				imu_calib_flag = imu.Read_IMU_Calib_Status();
				bool test = imu.Read_Calib_Params();

				if(imu_calib_flag){
					break;
				}

			}break;
			}
	}








	xSemaphoreGive(xInitializerMutex); //proceed into inf loop now that initialization is done


	while(1){

		xLastWakeTime = xTaskGetTickCount();
		vTaskDelayUntil(&xLastWakeTime, xFrequency); //blocks


		localState = imu.readIMU();

		xSemaphoreTake(xSharedStateMutex, (TickType_t) 0);
		*globalStateRef = localState;
		xSemaphoreGive(xSharedStateMutex);

	}

}








}
