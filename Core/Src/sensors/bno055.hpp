/*
 * bno055.hpp
 *
 *  Created on: Aug 21, 2022
 *      Author: danie
 */



#ifndef SRC_BNO055_HPP_
#define SRC_BNO055_HPP_

#include <stdbool.h>
#include <stdint.h>

#include "sensors.hpp"

//pin/port defines
#define I2C1_SCL_PORT GPIOB
#define I2C1_SDA_PORT GPIOB
#define I2C1_SCL_PIN GPIO_PIN_6
#define I2C1_SDA_PIN GPIO_PIN_7

//register defines
#define BNO055_ADDRESS 0x28
#define BNO055_CONFIG_R 0x3D
#define BNO055_CALIB_STAT_R 0x35
#define BNO055_ACC_CALIB_R 0x55
#define BNO055_GYR_CALIB_R 0x61
#define BNO055_ACC_DATA_R 0x08
#define BNO055_GYR_DATA_R 0x14

//accel data
#define BNO055_ACC_DATA_X_LSB_R 0x08
#define BNO055_ACC_DATA_X_MSB_R 0x09
#define BNO055_ACC_DATA_Y_LSB_R 0x0A
#define BNO055_ACC_DATA_Y_MSB_R 0x0B
#define BNO055_ACC_DATA_Z_LSB_R 0x0C
#define BNO055_ACC_DATA_Z_MSB_R 0x0D

//gyro data
#define BNO055_GYR_DATA_X_LSB_R 0x14
#define BNO055_GYR_DATA_X_MSB_R 0x15
#define BNO055_GYR_DATA_Y_LSB_R 0x16
#define BNO055_GYR_DATA_Y_MSB_R 0x17
#define BNO055_GYR_DATA_Z_LSB_R 0x18
#define BNO055_GYR_DATA_Z_MSB_R 0x19

//accel offsets
#define BNO055_ACC_OFFSET_X_LSB_R 0x55
#define BNO055_ACC_OFFSET_X_MSB_R 0x56
#define BNO055_ACC_OFFSET_Y_LSB_R 0x57
#define BNO055_ACC_OFFSET_Y_MSB_R 0x58
#define BNO055_ACC_OFFSET_Z_LSB_R 0x59
#define BNO055_ACC_OFFSET_Z_MSB_R 0x60

//gyro offsets
#define BNO055_GYR_OFFSET_X_LSB_R 0x61
#define BNO055_GYR_OFFSET_X_MSB_R 0x62
#define BNO055_GYR_OFFSET_Y_LSB_R 0x63
#define BNO055_GYR_OFFSET_Y_MSB_R 0x64
#define BNO055_GYR_OFFSET_Z_LSB_R 0x65
#define BNO055_GYR_OFFSET_Z_MSB_R 0x66

//value defines
#define IMU_OP_MODE 0x08
#define IMU_FULL_CALIB 0x3C

//scale defines
#define ACC_SCALE 100 //100 LSB = 1 m/s^2
#define GYR_SCALE 16 //16 LSB = 1 deg/s

namespace sensors{

class BNO055 : IMU{
public:
	BNO055(I2C_HandleTypeDef hi2c1);
	bool configSensor(void);
	bool Read_IMU_Calib_Status(void);
	bool Write_IMU_Calib_Params(void);
	bool Read_Calib_Params(void);
	state::QuadStateVector readIMU(void);
private:
	I2C_HandleTypeDef i2c;
	void I2C1_ClearBusyFlagErratum(I2C_HandleTypeDef *instance);
};

}

#endif
