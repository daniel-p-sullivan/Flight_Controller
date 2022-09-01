/*
 * transformers.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */


#include "transformers.hpp"
#include <cstdlib>
#include <vector>
#include <cmath>

namespace transformers{

std::vector<std::vector<float>> Rx(float phi){
	static std::vector<std::vector<float>> mat = {{1, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	mat[1][1] = std::cos(phi);
	mat[1][2] = -1 * std::sin(phi);
	mat[2][1] = std::sin(phi);
	mat[2][2] = std::cos(phi);
	return mat;
}

std::vector<std::vector<float>> Ry(float theta){
	static std::vector<std::vector<float>> mat = {{0, 0, 0}, {0, 1, 0}, {0, 0, 0}};
	mat[0][0] = std::cos(theta);
	mat[0][2] = std::sin(theta);
	mat[2][0] = -1 * std::sin(theta);
	mat[2][2] = std::cos(theta);
	return mat;
}

std::vector<std::vector<float>> Rz(float psi){
	static std::vector<std::vector<float>> mat = {{0, 0, 0}, {0, 0, 0}, {0, 0, 1}};
	mat[0][0] = std::cos(psi);
	mat[0][1] = -1 * std::sin(psi);
	mat[1][0] = std::sin(psi);
	mat[1][1] = std::cos(psi);
	return mat;
}

//std::vector<std::vector<float>>& bodyFrameToInertialFrame(float phi, float theta, float psi){
//	static std::vector<std::vector<float>> mat = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
//	std::vector<std::vector<float>> Rx = Rx(phi);
//	std::vector<std::vector<float>> Ry = Ry(theta);
//	std::vector<std::vector<float>> Rz = Rz(psi);
//	for(int i = 0; i < 3; i++){
//		for(int j = 0; j < 3; j++){
//				mat[i][j] = Rx[i][j] * Ry[i][j] * Rz[i][j];
//		}
//	}
//
//	return mat;
//}



}
