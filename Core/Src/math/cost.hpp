/*
 * cost.hpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */

#ifndef SRC_MATH_COST_HPP_
#define SRC_MATH_COST_HPP_

#include "../state/state.hpp"

namespace math{
namespace cost{


class CostFunction{
public:
	CostFunction(state::ControlTraj& ct);
	int eval();
private:
	state::ControlTraj myTraj;

};



}
}







#endif /* SRC_MATH_COST_HPP_ */
