/*
 * nrf24.hpp
 *
 *  Created on: Aug 21, 2022
 *      Author: danie
 */

#ifndef SRC_NRF24_HPP_
#define SRC_NRF24_HPP_

#include <cstdint>
#include "communications.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"

//register defines
#define CONFIG 	 	0x00
#define EN_AA		0x01
#define SETUP_RETR	0x04
#define RF_CH		0x05
#define RF_SETUP	0x06
#define STATUS 	 	0x07
#define RX_ADDR_P0	0x0A
#define RX_PW_P0 	0x11
#define FIFO_STATUS 0x17


//value defines
#define NRF24_PWR_UP 1
#define NRF24_PRIM_RX 1

//position defines
#define NRF24_PWR_UP_Pos 1
#define NRF24_PRIM_RX_Pos 0
#define RX_DR_Pos 6
#define RF_DR_LOW_Pos 5
#define RF_DR_HIGH_Pos 3

#define RX_DR_On (1 << RX_DR_Pos)

//width defines
#define RX_ADDR_P0_WIDTH 5

//setting defines
#define NRF24_PWR_UP_On  (NRF24_PWR_UP << NRF24_PWR_UP_Pos)
#define NRF24_PRIM_RX_On (NRF24_PRIM_RX << NRF24_PRIM_RX_Pos)

#define CHANNEL 2
#define PAYLOAD_SIZE 32  //number of bytes, needs to be less than 32

namespace communications{

class NRF24 : Communicator{
public:
	NRF24(SPI_HandleTypeDef spi);
//	int send(communications::comm_packet& msg);
//	int recv(communications::comm_packet& msg);
private:
	//spi handle
	SPI_HandleTypeDef* spi;

	//commands
	static const uint8_t R_RX_PAYLOAD = 0b01100001; //command to read the RX payload
	static const uint8_t W_REGISTER = 0b00100000;   //last 5 bits are the register map address
	static const uint8_t R_REGISTER = 0b00000000;	  //last 5 bits are the register map address
	static const uint8_t FLUSH_RX = 0b11100010;

	//payload transmit and receive buffers
	uint8_t payload_tx_buf[PAYLOAD_SIZE+1];
	static uint8_t payload_data_buf[PAYLOAD_SIZE+1];
	static uint8_t data[PAYLOAD_SIZE+1];


	//configuration
	uint8_t p0_address[RX_ADDR_P0_WIDTH];

	enum nrf24_speed{
		nrf24_1Mbps = 0,
		nrf24_2Mbps = 1,
		nrf24_250kbps = 2
	};

	static const enum nrf24_speed nrf24_comm_speed = nrf24_1Mbps;

	void Read_Register(uint8_t register_address, uint8_t* data);
	void Write_Register(uint8_t register_address, uint8_t* data);
	void Read_MB_Register(uint8_t register_address, uint8_t* data, uint8_t len);
	void Write_MB_Register(uint8_t register_address, uint8_t* data, uint8_t len);
	void Set_Payload_Size(uint8_t size);
	void Read_Payload(void);
	void Enable_Receive(void);
	void Set_Data_Rate(enum nrf24_speed s);
	void Send_Command(uint8_t command);
	void Begin_SPI(void);
	void End_SPI(void);
	void Flush_RX(void);
	void Clear_FIFO_Interrupt(void);
	bool Payload_Available(void);
	bool array_eq(uint8_t* arr1, uint8_t* arr2, uint8_t len);

};

}

#endif
