/*
 * trajectories.hpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */

#ifndef SRC_STATE_TRAJECTORIES_HPP_
#define SRC_STATE_TRAJECTORIES_HPP_

#include "state.hpp"

namespace state{

class Trajectory{
public:
	vector<State>& get_trajectory();
private:
	vector<State> traj;
};

class ControlTraj : Trajectory{
public:
	vector<action>& getControlTrajectory();
private:
	vector<action> controlActions;
};


}



#endif /* SRC_STATE_TRAJECTORIES_HPP_ */
