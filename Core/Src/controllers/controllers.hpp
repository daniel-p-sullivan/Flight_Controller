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
#include "../math/integrators.hpp"
#include "../math/differentiators.hpp"

#define INTEGRAL_WINDUP_MAX 200
#define INTEGRAL_WINDUP_MIN -200

namespace control{

class PI{
public:
	PI(float sp, float dt, float Kp, float Ki);
	float calcOutput(float stateEstimate);
private:
	math::TrapezoidalIntegrator errorIntegrator;
	float setpoint;
	float dt;
	float Kp;
	float Ki;
};

class PID{
public:
	PID(float sp, float dt, float Kp, float Ki, float Kd);
	float calcOutput(float stateEstimate);
private:
	math::TrapezoidalIntegrator errorIntegrator;
	math::BackwardsDifferentiator errorDifferentiator;
	float setpoint;
	float dt;
	float Kp;
	float Ki;
	float Kd;
};

};

#endif /* SRC_CONTROLLERS_HPP_ */
