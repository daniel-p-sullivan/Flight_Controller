/*
 * transformers.hpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */

#ifndef SRC_MATH_TRANSFORMERS_HPP_
#define SRC_MATH_TRANSFORMERS_HPP_

namespace transformers{


vector<vector<float>>& Rx(float psi);
vector<vector<float>>& Ry(float theta);
vector<vector<float>>& Rz(float psi);
vector<vector<float>>& bodyFrameToInertialFrame(float phi, float theta, float psi);

}

#endif /* SRC_MATH_TRANSFORMERS_HPP_ */
