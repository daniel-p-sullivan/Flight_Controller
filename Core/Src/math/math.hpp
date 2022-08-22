/*
 * math.hpp
 *
 *  Created on: Aug 21, 2022
 *      Author: danie
 */

#ifndef SRC_MATH_MATH_HPP_
#define SRC_MATH_MATH_HPP_

#include <cstdint>

namespace math{

class Differentiator{
public:
	virtual float calculate_derivative(Eigen::Vector1d meas);
};

class BackwardsDiff : Differentiator{

public:
	float calculate_derivative(Eigen::Vector1d meas);
private:
	Eigen::Vector1d past_meas;
};

class BackwardsDiffMultiSample : Differentiator{
public:
	BackwardsDiffMultiSample(int num_samples);
	float calculate_derivative(Eigen::Vector1d meas);
private:
	uint16_t num_samples;
	Eigen::Vector1d past_meas;
};

}






#endif /* SRC_MATH_MATH_HPP_ */
