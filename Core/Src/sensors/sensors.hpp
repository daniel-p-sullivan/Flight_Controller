/*
 * sensors.h
 *
 *  Created on: May 30, 2021
 *      Author: danie
 */

#ifndef SRC_SENSORS_HPP_
#define SRC_SENSORS_HPP_

#include "main.h"
#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>


typedef struct {
	int16_t a_x, a_y, a_z, g_x, g_y, g_z;
} IMU_Sample;

extern I2C_HandleTypeDef hi2c1;
static HAL_StatusTypeDef hal_status;

class IMU{
public:
	virtual bool Config_Sensor(void);
	virtual bool Read_IMU_Calib_Status(void);
	virtual bool Write_IMU_Calib_Params(void);
	virtual bool Read_Calib_Params(void);
	virtual IMU_Sample* Read_IMU(void);
	virtual void Update_IMU_Sample(IMU_Sample* sample);
private:
	virtual void I2C1_ClearBusyFlagErratum(I2C_HandleTypeDef *instance);
};





#endif /* SRC_SENSORS_HPP_ */
