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
#include "../state/state.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"

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

//mag offsets
#define BNO055_MAG_OFFSET_X_LSB_R 0x5B
#define BNO055_MAG_OFFSET_X_MSB_R 0x5C
#define BNO055_MAG_OFFSET_Y_LSB_R 0x5D
#define BNO055_MAG_OFFSET_Y_MSB_R 0x5E
#define BNO055_MAG_OFFSET_Z_LSB_R 0x5F
#define BNO055_MAG_OFFSET_Z_MSB_R 0x60

//radius configuration registers
#define BNO055_ACC_RADIUS_LSB_R 0x67
#define BNO055_ACC_RADIUS_MSB_R 0x68
#define BNO055_MAG_RADIUS_LSB_R 0x69
#define BNO055_MAG_RADIUS_MSB_R 0x6A

//absolute orientation
#define BNO055_EUL_PITCH_MSB_R 0x1F
#define BNO055_EUL_PITCH_LSB_R 0x1E
#define BNO055_EUL_ROLL_MSB_R 0x1D
#define BNO055_EUL_ROLL_LSB_R 0x1C
#define BNO055_EUL_HEADING_MSB_R 0x1B
#define BNO055_EUL_HEADING_LSB_R 0x1A

//absolute accels
#define BNO055_LIA_DATA_Z_MSB_R 0x2D
#define BNO055_LIA_DATA_Z_LSB_R 0x2C

//value defines
#define IMU_OP_MODE 0x08
#define NDOF_MODE 0x0C
#define IMU_FULL_CALIB 0x3C
#define NDOF_FULL_CALIB 0x3F

//scale defines
#define ACC_SCALE 100 //100 LSB = 1 m/s^2
#define GYR_SCALE 16 //16 LSB = 1 deg/s
#define EUL_SCALE 16 //16 LSB = 1 deg

namespace sensors{

class BNO055 : IMU{
public:
	BNO055(I2C_HandleTypeDef& rhi2c1);
	bool configSensor(void);
	bool readCalibStatus(void);
	state::QuadStateVector& readIMU(void);
private:
	I2C_HandleTypeDef& i2c;
	bool writeCalibParams(void);
	bool readCalibParams(void);
	void clearBusyFlagErratum(void);
};

}

#endif
