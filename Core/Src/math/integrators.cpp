/*
 * integrators.cpp
 *
 *  Created on: Sep 18, 2022
 *      Author: danie
 */


#include "integrators.hpp"

namespace math{

TrapezoidalIntegrator::TrapezoidalIntegrator(float dt) :
		  dt(dt), currVal(0), prevVal(0){}

float TrapezoidalIntegrator::integrate(float input){

	currIntegral += (prevVal + input) * dt / 2;
	prevVal = input;
	return currIntegral;

}

}

