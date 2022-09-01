/*
 * trajectories.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */

#include <vector>
#include "state.hpp"

namespace state{

class Trajectory{
public:
	std::vector<state::QuadStateVector> get_trajectory(){
		return this->traj;
	}
private:
	std::vector<state::QuadStateVector> traj;
};




}
