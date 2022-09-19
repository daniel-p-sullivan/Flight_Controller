/*
 * lowpassfilter.cpp
 *
 *  Created on: Sep 18, 2022
 *      Author: danie
 */


#include "lowpassfilters.hpp"

namespace estimators{

MovingAverageFilter::MovingAverageFilter(int size)
		: movingAverage(0), size(size){}

float MovingAverageFilter::filter(float input){

	movingAverage -= movingAverage / size;
	movingAverage += input / size;

	return movingAverage;



}



}
