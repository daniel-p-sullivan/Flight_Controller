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

namespace control{

class Controller{
public:
	virtual void set_setpoint(state::Setpoint sp);
	virtual state::Setpoint calc_output(state::State st);
};

class RpyController : Controller{
public:
	void set_setpoint(state::TrpySetpoint sp);
	state::RpySetpoint calc_output(state::TrpyState st);
private:
	float Kp;
	float Ki;
	float dt;
	state::RpySetpoint sp;
	state::RpyState meas;
	state::RpyError err;
	state::RpyError integ_err;
};





};

#endif /* SRC_CONTROLLERS_HPP_ */
