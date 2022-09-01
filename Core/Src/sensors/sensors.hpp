/*
 * sensors.h
 *
 *  Created on: May 30, 2021
 *      Author: danie
 */

#ifndef SRC_SENSORS_HPP_
#define SRC_SENSORS_HPP_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "../state/state.hpp"

#include <stdbool.h>
#include <cstdint>

namespace sensors{


class IMU{
public:
	virtual bool configSensor(void);
	virtual state::QuadStateVector& readIMU(void);

};

}
#endif /* SRC_SENSORS_HPP_ */
