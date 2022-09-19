/*
 * differentiators.cpp
 *
 *  Created on: Sep 18, 2022
 *      Author: danie
 */

#include "differentiators.hpp"

namespace math{

BackwardsDifferentiator::BackwardsDifferentiator(float dt)
		: dt(dt), prevVal(0){}

float BackwardsDifferentiator::differentiate(float input){

	float deriv = (input - prevVal) / dt;
	prevVal = input;
	return deriv;

}


}

