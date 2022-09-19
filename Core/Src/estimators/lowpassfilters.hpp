/*
 * lowpassfilter.hpp
 *
 *  Created on: Sep 18, 2022
 *      Author: danie
 */

#ifndef SRC_ESTIMATORS_LOWPASSFILTERS_HPP_
#define SRC_ESTIMATORS_LOWPASSFILTERS_HPP_


namespace estimators{

class MovingAverageFilter{
public:
	MovingAverageFilter(int size);
	float filter(float input);
private:
	float movingAverage;
	int size;
};



}






#endif /* SRC_ESTIMATORS_LOWPASSFILTERS_HPP_ */
