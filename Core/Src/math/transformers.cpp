/*
 * transformers.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */


#include "transformers.hpp"
#include <cstdlib>

namespace transformers{

vector<vector<float>>& Rx(float phi){
	static vector<vector<float>> mat = {{1, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	mat[1][1] = std::cos(phi);
	mat[1][2] = -1 * std::sin(phi);
	mat[2][1] = std::sin(phi);
	mat[2][2] = std::cos(phi);
	return mat;
}

vector<vector<float>>& Ry(float theta){
	static vector<vector<float>> mat = {{0, 0, 0}, {0, 1, 0}, {0, 0, 0}};
	mat[0][0] = std::cos(theta);
	mat[0][2] = std::sin(theta);
	mat[2][0] = -1 * std::sin(theta);
	mat[2][2] = std::cos(theta);
	return mat;
}

vector<vector<float>>& Rz(float psi){
	static vector<vector<float>> mat = {{0, 0, 0}, {0, 0, 0}, {0, 0, 1}};
	mat[0][0] = std::cos(psi);
	mat[0][1] = -1 * std::sin(psi);
	mat[1][0] = std::sin(psi);
	mat[1][1] = std::cos(psi);
	return mat;
}

vector<vector<float>>& bodyFrameToInertialFrame(float phi, float theta, float psi){
	static vector<vector<float>> mat = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	static vector<vector<float>> Rx = Rx(phi);
	static vector<vector<float>> Ry = Ry(theta);
	static vector<vector<float>> Rz = Rz(psi);
	for(int i = 0; i < mat.size(); i++){
		for(int j = 0; j < mat.size(); j++){
				mat[i][j] = Rx[i][j] * Ry[i][j] * Rz[i][j];
		}
	}

	return mat;
}



}
