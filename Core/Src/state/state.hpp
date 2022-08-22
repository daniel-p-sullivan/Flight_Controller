/*
 * state.hpp
 *
 *  Created on: Aug 21, 2022
 *      Author: danie
 */

#ifndef SRC_STATE_STATE_HPP_
#define SRC_STATE_STATE_HPP_


namespace state{

typedef struct trpy_state{
	float roll;
	float pitch;
	float yaw;
	float thrust;
} trpy_state;

typedef struct xyz_state{
	float x;
	float y;
	float z;
	float dx;
	float dy;
	float dz;
} xyz_state;

typedef xyz_state xyz_setpoint;
typedef trpy_state trpy_setpoint;
class setpoint : xyz_setpoint, trpy_setpoint{};

}

#endif /* SRC_STATE_STATE_HPP_ */
