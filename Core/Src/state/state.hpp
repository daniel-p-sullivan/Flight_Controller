/*
 * state.hpp
 *
 *  Created on: Aug 21, 2022
 *      Author: danie
 */

#ifndef SRC_STATE_STATE_HPP_
#define SRC_STATE_STATE_HPP_


namespace state{

struct RpyState{
	float roll;
	float pitch;
	float yaw;
	float z;
};

struct XyzState{
	float x;
	float y;
	float z;
	float dx;
	float dy;
	float dz;
};

typedef XyzState XyzSetpoint;
typedef XyzState XyzError;
typedef RpyState RpySetpoint;
typedef RpyState RpyError;
class State : XyzState, RpyState{};
class Setpoint : XyzSetpoint, RpySetpoint{};


}

#endif /* SRC_STATE_STATE_HPP_ */
