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

#define INTEGRAL_WINDUP_MAX 200
#define INTEGRAL_WINDUP_MIN -200

namespace control{

class PI{
public:
	PI(float sp, float dt, float Kp, float Ki);
	float calcOutput(float stateEstimate);
private:
	float setpoint;
	float dt;
	float Kp;
	float Ki;
};


};

#endif /* SRC_CONTROLLERS_HPP_ */
