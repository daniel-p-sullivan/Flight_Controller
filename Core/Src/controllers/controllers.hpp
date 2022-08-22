/*
 * controllers.h
 *
 *  Created on: Jun 9, 2021
 *      Author: danie
 */

#ifndef SRC_CONTROLLERS_HPP_
#define SRC_CONTROLLERS_HPP_

#include "../actuators/actuators.hpp"
#include "../sensors/sensors.hpp"
#include "../state/state.hpp"

//#define Kp -1
#define Ki 0
#define Kd 1

static const float Kp = -1;
//static const float Ki = 0.01;
namespace control{

class Controller{
public:
	virtual void set_setpoint(state::setpoint s);
};

class PIController : Controller{
public:
	void set_setpoint(state::setpoint s);
};

typedef struct {
	int16_t sp;
	int16_t meas;
	int16_t error;
	int16_t integral_error;
	int16_t output;
	uint8_t dt;
} PI;

typedef struct{
	int16_t roll_setpoint;  //  deg/s
	int16_t pitch_setpoint; //  deg/s
	int16_t yaw_setpoint;	//   deg/s
	int16_t z_setpoint;	//	 m/s^2
} TRPY_Setpoint;

typedef struct {
	int16_t roll_output;
	int16_t pitch_output;
	int16_t yaw_output;
	int16_t thrust_output;
} TRPY_Output;

typedef struct {
	PI* roll_PI;
	PI* pitch_PI;
	PI* yaw_PI;
	PI* thrust_PI;
	TRPY_Output* trpy_o;
	int8_t dt;
} TRPY_Controller;


PI* Construct_PI(uint8_t dt);
void Iter_PI(PI *self);

TRPY_Controller* Construct_TRPY_Controller(int8_t dt);
void Update_Sample(TRPY_Controller *self, IMU_Sample* sample);
void Update_Setpoint(TRPY_Controller *self, TRPY_Setpoint* trpy_sp);
void Iter_TRPY_Controller(TRPY_Controller* self, IMU_Sample *sample);

void Motor_Mixing_Algorithm(TRPY_Output *trpy_o);
uint16_t pos(int16_t x);

};

#endif /* SRC_CONTROLLERS_HPP_ */
