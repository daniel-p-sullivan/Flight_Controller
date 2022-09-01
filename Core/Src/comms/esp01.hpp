/*
 * esp01.hpp
 *
 *  Created on: Aug 30, 2022
 *      Author: danie
 */

#ifndef SRC_COMMS_ESP01_HPP_
#define SRC_COMMS_ESP01_HPP_

#include "communications.hpp"

namespace communications{


class ESP01 : Communicator{
public:
	ESP01(SPI_HandleTypeDef spi);
	int send(communications::comm_packet& msg);
	int recv(communications::comm_packet& msg);
private:
	SPI_HandleTypeDef* spi;


}

}

#endif /* SRC_COMMS_ESP01_HPP_ */
