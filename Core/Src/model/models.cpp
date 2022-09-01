/*
 * models.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */

#include "models.hpp"
#include <cstdlib>
#include <cmath>

namespace models{


//Quadcotpter::Quadcopter(state::QuadStateVector newState){
//		this->currState = newState;
//		this->currStateDeriv = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//	}
//
//	void Quadcopter::predict(state::QuadControlActions newAction){
//
//		this->currStateDeriv.dphi = this->currState.dphi;
//		this->currStateDeriv.ddphi = this->a1 * this->currState.dtheta * this->currState.dpsi + this->b1 * newAction.u2;
//		this->currStateDeriv.dtheta = this->currState.dtheta;
//		this->currStateDeriv.ddtheta = this->a2 * this->currState.dpsi * this->currState.dpsi + this->b2 * newAction.u3;
//		this->currStateDeriv.dpsi = this->currState.dpsi;
//		this->currStateDeriv.ddpsi = this->a3 * this->currState.dpsi * this->currState.dtheta + this->b3 * newAction.u4;
//		this->currStateDeriv.dx = this->currState.dx;
//		this->currStateDeriv.ddx = (this->u1 / this->m) * (std::cos(this->currState.phi) * std::sin(this->currState.theta) * std::cos(this->currState.psi) + std::sin(this->currState.phi) * std::sin(this->currState.psi));
//		this->currStateDeriv.dy = this->currState.dy;
//		this->currStateDeriv.ddy = (this->u1 / this->m) * (std::cos(this->currState.phi) * std::sin(this->currState.theta) * std::sin(this->currState.psi) - std::sin(this->currState.phi) * std::cos(this->currState.psi));
//		this->currStateDeriv.dz = this->currState.dz;
//		this->currStateDeriv.ddz = (this->u1 / this->m) * (std::cos(this->currState.phi) * std::cos(this->currState.theta)) - this->g;
//
//
//
//	}







}
