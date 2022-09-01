/*
 * trajectories.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */


namespace state{

class Trajectory{
public:
	vector<state> get_trajectory(){
		return this->traj;
	}
private:
	vector<state> traj;
};




}
