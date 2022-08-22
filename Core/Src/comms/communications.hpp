/*
 * communications.h
 *
 *  Created on: Jun 10, 2021
 *      Author: danie
 */

#ifndef SRC_COMMUNICATIONS_HPP_
#define SRC_COMMUNICATIONS_HPP_

#include "../controllers/controllers.hpp"
//#include "main.h"



#define PAYLOAD_SIZE 32


namespace communications{


typedef enum command{
	turn_off = 0,
	setpoint = 1,
} command;

typedef struct comm_packet{
	command cmd;
	char payload[PAYLOAD_SIZE-1];
} comm_packet;

class Communicator{
public:
	virtual int send(comm_packet& msg);
	virtual int recv(comm_packet& msg);
};


}

#endif /* SRC_COMMUNICATIONS_HPP_ */