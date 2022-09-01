/*
 * transformers.hpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */

#ifndef SRC_MATH_TRANSFORMERS_HPP_
#define SRC_MATH_TRANSFORMERS_HPP_

#include <vector>

namespace transformers{


std::vector<std::vector<float>> Rx(float psi);
std::vector<std::vector<float>> Ry(float theta);
std::vector<std::vector<float>> Rz(float psi);
std::vector<std::vector<float>>& bodyFrameToInertialFrame(float phi, float theta, float psi);

}

#endif /* SRC_MATH_TRANSFORMERS_HPP_ */
