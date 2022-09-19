/*
 * integrators.hpp
 *
 *  Created on: Sep 18, 2022
 *      Author: danie
 */

#ifndef SRC_MATH_INTEGRATORS_HPP_
#define SRC_MATH_INTEGRATORS_HPP_


namespace math{

class TrapezoidalIntegrator{
public:
	TrapezoidalIntegrator(float dt);
	float integrate(float input);
private:
	float dt;
	float currIntegral;
	float currVal;
	float prevVal;
};

}


#endif /* SRC_MATH_INTEGRATORS_HPP_ */
