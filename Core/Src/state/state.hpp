/*
 * state.hpp
 *
 *  Created on: Aug 21, 2022
 *      Author: danie
 */

#ifndef SRC_STATE_STATE_HPP_
#define SRC_STATE_STATE_HPP_

#include <stdint.h>

namespace state{

struct QuadStateVector{
	float phi;
	float dphi;
	float theta;
	float dtheta;
	float psi;
	float dpsi;
	float x;
	float dx;
	float y;
	float dy;
	float z;
	float dz;
};


struct QuadControlActions{
	float u1;
	float u2;
	float u3;
	float u4;
};






struct QuadStateDerivVector{
	float dphi;
	float ddphi;
	float dtheta;
	float ddtheta;
	float dpsi;
	float ddpsi;
	float dx;
	float ddx;
	float dy;
	float ddy;
	float dz;
	float ddz;
};





//
//class State{
//public:
//	virtual vector<float> get_state();
//	virtual void set_state(vector<float>);
//private:
//	virtual vector<float> state;
//};
//
//class RpyState : State{
//public:
//	vector<float> get_state();
//	void set_state(vector<float>);
//private:
//	vector<float> state;
//};
//
//class XyzState : State{
//public:
//	vector<float> get_state();
//	void set_state(vector<float>);
//private:
//	vector<float> state;
//};
//
//
//class Setpoint{
//public:
//	virtual void set_setpoint(vector<float> setpoint);
//	virtual vector<float> get_setpoint();
//private:
//	vector<float> setpoint;
//};
//
//
//class RpySetpoint : Setpoint{
//public:
//	vector<float> get_setpoint();
//	void set_serpoint(vector<float>);
//private:
//	vector<float> setpoint;
//};
//
//class XyzSetpoint : Setpoint{
//public:
//	vector<float> get_setpoint();
//	void set_setpoint(vector<float>);
//private:
//	vector<float> setpoint;
//};
//
//
//typedef XyzState XyzError;
//typedef RpyState RpyError;



}

#endif /* SRC_STATE_STATE_HPP_ */
