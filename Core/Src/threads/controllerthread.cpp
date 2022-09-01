/*
 * controllerthread.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */

#include "../controllers/controllers.hpp"
#include "../state/state.hpp"


namespace threads{

void controllerThread(state::QuadStateVector& stateEstimate, state::QuadControlActions& output){


	control::PI thrustController = control::PI(0, 0.1, 10, 2);
	control::PI yawController = control::PI(0, 0.1, 10, 2);
	control::PI rollController = control::PI(0, 0.1, 10, 2);
	control::PI pitchController = control::PI(0, 0.1, 10, 2);
	state::QuadStateVector estimate;
	state::QuadControlActions calcedOutput;

	while(1){

		//get measurement
		//lock(stateEstimate)
		estimate = stateEstimate;
		//unlock(stateEstimate)


		calcedOutput.u1 = thrustController.calcOutput(estimate.z);
		calcedOutput.u2 = rollController.calcOutput(estimate.psi);
		calcedOutput.u3 = pitchController.calcOutput(estimate.theta);
		calcedOutput.u4 = yawController.calcOutput(estimate.phi);

		//lock(output)
		output = calcedOutput;
		//unlock(output)



	}


}

}
