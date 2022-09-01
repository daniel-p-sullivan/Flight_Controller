/*
 * actions.hpp
 *
 *  Created on: Aug 31, 2022
 *      Author: danie
 */

#ifndef SRC_STATE_ACTIONS_HPP_
#define SRC_STATE_ACTIONS_HPP_

namespace state{

class Actions{
public:
	virtual vector<float> get_actions();
	virtual void set_actions(vector<float>);
private:
	virtual vector<float> actions;

};


}







#endif /* SRC_STATE_ACTIONS_HPP_ */
