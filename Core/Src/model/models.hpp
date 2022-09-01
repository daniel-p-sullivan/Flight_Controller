/*
 * models.hpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */

#ifndef SRC_MODEL_MODELS_HPP_
#define SRC_MODEL_MODELS_HPP_

#include "../state/state.hpp"

namespace models{



class Quadcopter{
public:
	Quadcopter(state::QuadStateVector newState);
	void predict(state::QuadControlActions newAction);
private:
	//coefficients
	const float a1;
	const float a2;
	const float a3;
	const float b1;
	const float b2;
	const float b3;
	const float m;
	const float g = 9.81;

	//explicit 12 dof state vars
	state::QuadStateVector currState;
	state::QuadStateDerivVector currStateDeriv;
	state::Setpoint currSetpoint;
	vector<state::State> currState;
	vector<state::Actions> currActions;
};



}







#endif /* SRC_MODEL_MODELS_HPP_ */
