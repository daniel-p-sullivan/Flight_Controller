/*
 * nrf24.cpp
 *
 *  Created on: Aug 21, 2022
 *      Author: danie
 */

/*
 * communications.cpp
 *
 *  Created on: Jun 10, 2021
 *      Author: danie
 */

#include "nrf24.hpp"

namespace communications{

NRF24::NRF24(SPI_HandleTypeDef spi){

	//Save the SPI handle
	this->spi = &spi;

	//CONFIGURE THE payload_tx_buf TO ALLOW FOR PAYLOAD READING
	this->payload_tx_buf[0] = this->R_RX_PAYLOAD;
	for(int i = 1; i < PAYLOAD_SIZE+1; i++){
			this->payload_tx_buf[i] = 0xff; //pad rest of the buffer with NOPs
	}

	//set CE low before configuring the NRF?
	HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET);

	//CSN must be high, transitioning to low starts SPI
	End_SPI();

	//DISABLE AUTO-ACK
	uint8_t disable_autoack = 0x00;
	uint8_t disable_autoretransmit = 0x00;
	Write_Register(EN_AA, &disable_autoack);
	Write_Register(SETUP_RETR, &disable_autoretransmit);


	//SET THE PAYLOAD SIZE
	uint8_t size_readback;
	while(size_readback != PAYLOAD_SIZE){
		//write the payload size and verify
		Set_Payload_Size(PAYLOAD_SIZE);
		HAL_Delay(3);
		Read_Register((uint8_t)RX_PW_P0, &size_readback);
	}


	//CONFIGURE THE ADDRESS FOR PIPE0
	uint8_t p0_address_readback[RX_ADDR_P0_WIDTH];
	while(!array_eq(p0_address_readback, this->p0_address, (uint8_t)RX_ADDR_P0_WIDTH)){
		//write the address for pipe0, verify it's correct
		Write_MB_Register((uint8_t)RX_ADDR_P0, this->p0_address, sizeof(this->p0_address));
		HAL_Delay(3);
		Read_MB_Register((uint8_t)RX_ADDR_P0, &p0_address_readback, sizeof(this->p0_address));
	}


	//SET THE COMMUNICATION SPEED
	Set_Data_Rate(this->nrf24_comm_speed);

	//POWER ON AND SET TO PRIM_RX MODE
	uint8_t config_data = 0x00 | NRF24_PWR_UP_On | NRF24_PRIM_RX_On;
	uint8_t config_readback = 0x00;
	while(config_readback != config_data){
		//write the config and verify
		Write_Register((uint8_t)CONFIG, &config_data);
		HAL_Delay(3);
		Read_Register((uint8_t)CONFIG, &config_readback);

	}

}

void NRF24::Read_Register(uint8_t register_address, uint8_t* data){

	static uint8_t command;
	command = this->R_REGISTER | register_address;

	//bring CSN low to select the comms
	Begin_SPI();

	HAL_StatusTypeDef hal_status = HAL_SPI_Transmit(this->spi, &command, 1, (uint32_t)10);
	hal_status = HAL_SPI_Receive(this->spi, data, 1, (uint32_t)10);

	//deselect the comms by setting CSN high
	End_SPI();


}

void NRF24::Write_Register(uint8_t register_address, uint8_t* data){
	static uint8_t command;
	command = this->W_REGISTER | register_address;

	//bring CSN low to select the comms
	Begin_SPI();

	HAL_StatusTypeDef hal_status = HAL_SPI_Transmit(this->spi, &command, 1, (uint32_t)10);
	hal_status = HAL_SPI_Transmit(this->spi, data, 1, (uint32_t)10);

	//deselect the comms by setting CSN high
	End_SPI();


}

void NRF24::Read_MB_Register(uint8_t register_address, uint8_t* buf, uint8_t len){

	static uint8_t command;
	command = this->R_REGISTER | register_address;

	//bring CSN low to select the comms
	Begin_SPI();

	HAL_StatusTypeDef hal_status = HAL_SPI_Transmit(this->spi, &command, 1, (uint32_t)10);
	while(len--){
		hal_status = HAL_SPI_Receive(this->spi, buf++, 1, (uint32_t)10);
	}

	//deselect the comms by setting CSN high
	End_SPI();



}


void NRF24::Write_MB_Register(uint8_t register_address, uint8_t* data, uint8_t len){

	static uint8_t command;
	command = this->W_REGISTER | register_address;

	//bring CSN low to select the comms
	Begin_SPI();

	HAL_StatusTypeDef hal_status = HAL_SPI_Transmit(this->spi, &command, 1, (uint32_t)10);
	while(len--){
		hal_status = HAL_SPI_Transmit(this->spi, data++, 1, (uint32_t)10);
	}

	//deselect the comms by setting CSN high
	End_SPI();


}


void NRF24::Read_Payload(void){

	static uint8_t fifo_status;


	//bring CSN low to select the comms
	Begin_SPI();


	HAL_StatusTypeDef hal_status = HAL_SPI_TransmitReceive(this->spi, payload_tx_buf, payload_data_buf, PAYLOAD_SIZE+1, (uint32_t)1000);
//	hal_status = HAL_SPI_Transmit(this->spi, &this->R_RX_PAYLOAD, 1, (uint32_t)10);
//	for(int i = 0; i < PAYLOAD_SIZE+1; i++){
//		hal_status = HAL_SPI_Receive(this->spi, &data[i], 1, (uint32_t)10);
//	}

	//deselect the comms by setting CSN high
	End_SPI();

	//clear the RX_DR interrupt
	Clear_FIFO_Interrupt();

	//read fifo status and check for more payloads, if there are, "repeat from step 1"
	Read_Register(FIFO_STATUS, &fifo_status);
	if((fifo_status & 0x01) == 0){
		Read_Payload();
	}

}

void NRF24::Set_Payload_Size(uint8_t size){

	//5 total pipes to set to the correct size
	for(int i = 0; i < 6; i++){
		Write_Register((uint8_t)(RX_PW_P0 + i), &size);
	}

}

void NRF24::Enable_Receive(void){
	//set CE high to enter RX mode
	HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_SET);
}

void NRF24::Send_Command(uint8_t command){

	//bring CSN low to select the comms
	Begin_SPI();

	HAL_SPI_Transmit(this->spi, &command, 1, (uint32_t)100);

	//deselect the comms by setting CSN high
	End_SPI();

}

void NRF24::Set_Data_Rate(enum nrf24_speed s){

	uint8_t dr_low = (s & 0x02) >> 1;
	uint8_t dr_high = s & 0x01;
	uint8_t data = (dr_low << RF_DR_LOW_Pos) | (dr_high << RF_DR_HIGH_Pos);
	Write_Register((uint8_t)RF_SETUP, &data);

}

void NRF24::Begin_SPI(void){
	//bring CSN low to select the comms
	HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
}
void NRF24::End_SPI(void){
	//deselect the comms by setting CSN high
	HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}

void NRF24::Flush_RX(void){
	//flush RX fifo
	Send_Command(this->FLUSH_RX);
}

void NRF24::Clear_FIFO_Interrupt(void){
	//clear the interrupt
	static uint8_t tdata = RX_DR_On;
	Write_Register(STATUS, &tdata);
}

bool NRF24::Payload_Available(void){
	//read the NRF24_RX_DR input pin
	if(HAL_GPIO_ReadPin(NRF24_RX_DR_GPIO_Port, NRF24_RX_DR_Pin) == GPIO_PIN_RESET){ //irq is active low
		return true;
	}else{
		return false;
	}
}

bool NRF24::array_eq(uint8_t* arr1, uint8_t* arr2, uint8_t len){
	while(len--){
		if(*(arr1++) != *(arr2++)){
			return false;
		}
	}

	return true;
}

}


