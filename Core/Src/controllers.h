/*
 * controllers.h
 *
 *  Created on: Jun 9, 2021
 *      Author: danie
 */

#ifndef SRC_CONTROLLERS_H_
#define SRC_CONTROLLERS_H_

#include "sensors.h"
#include "actuators.h"

#define Kp 10
#define Ki 10
#define Kd 1

#define MOTOR_IDLE 1337

static const float dt_control_loop = 0.001;

typedef struct{
	uint16_t roll_setpoint;  //  deg/s
	uint16_t pitch_setpoint; //  deg/s
	uint16_t yaw_setpoint;	//   deg/s
	uint16_t z_setpoint;	//	 m/s^2
} Setpoint;

typedef struct{
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
} Attitude;

int32_t PI(int16_t *sp, int16_t* meas, int16_t *i_error);
void trpy_Controller(Setpoint *sp, IMU_sample *sample);
void Motor_Mixing_Algorithm(int16_t *thrust, int16_t *roll, int16_t *pitch, int16_t *yaw);

#endif /* SRC_CONTROLLERS_H_ */
