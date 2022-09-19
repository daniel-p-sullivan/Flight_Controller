/*
 * differentiators.hpp
 *
 *  Created on: Sep 18, 2022
 *      Author: danie
 */

#ifndef SRC_MATH_DIFFERENTIATORS_HPP_
#define SRC_MATH_DIFFERENTIATORS_HPP_


namespace math{

class BackwardsDifferentiator{
public:
	BackwardsDifferentiator(float dt);
	float differentiate(float input);
private:
	float prevVal;
	float dt;
};







}






#endif /* SRC_MATH_DIFFERENTIATORS_HPP_ */
