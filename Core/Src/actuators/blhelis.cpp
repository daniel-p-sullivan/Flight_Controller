/*
 * blhelis.cpp
 *
 *  Created on: Aug 21, 2022
 *      Author: danie
 */


#include "blhelis.hpp"
#include "../state/state.hpp"

namespace actuators{

BLHelis::BLHelis(TIM_HandleTypeDef& rhtim8) : timer(rhtim8){}


void BLHelis::Init_Motors(void){
	HAL_TIM_Base_Start(&(this->timer));
	HAL_TIM_PWM_Start(&(this->timer), TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&(this->timer), TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&(this->timer), TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&(this->timer), TIM_CHANNEL_4);
	static uint16_t mc1_l = MOTOR_1MS;
	static uint16_t mc1_h = MOTOR_13MS;
	static uint16_t mc1 = 0;
	static uint16_t step_count = 1000;
	static uint8_t delay = 10;
	float step = (mc1_h - mc1_l) / step_count;

	motor_sp msp;

	//delay for motor startup????
	//vTaskDelay(10000);
	for(uint16_t i = 0; i < (step_count/2); i++){
		mc1 = mc1_l + i * (int)step;
		msp = {mc1, mc1, mc1, mc1};
		this->Update_Motor_SP(msp);
		vTaskDelay(delay);
	}
	for(uint16_t i = 0; i < (step_count/2); i++){
		mc1 = mc1_h - i * (int)step;
		msp = {mc1, mc1, mc1, mc1};
		this->Update_Motor_SP(msp);
		vTaskDelay(delay);
	}
	mc1 = mc1_l;
	msp = {mc1, mc1, mc1, mc1};
	this->Update_Motor_SP(msp);
	vTaskDelay(2000);
}

void BLHelis::actuateMotors(state::QuadControlActions& ac){

	static uint16_t m1_sp, m2_sp, m3_sp, m4_sp;

	//mix the controller output
	m1_sp = ac.u1 + ac.u2 + ac.u3 + ac.u4;
	m2_sp = ac.u1 - ac.u2 + ac.u3 - ac.u4;
	m3_sp = ac.u1 + ac.u2 - ac.u3 - ac.u4;
	m4_sp = ac.u1 - ac.u2 - ac.u3 + ac.u4;

	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_1, m1_sp);
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_2, m2_sp);
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_3, m3_sp);
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_4, m4_sp);
}


void BLHelis::Actuate_Motor_1(void){
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_1, MOTOR_IDLE);
}

void BLHelis::Actuate_Motor_2(void){
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_2, MOTOR_IDLE);
}
void BLHelis::Actuate_Motor_3(void){
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_3, MOTOR_IDLE);
}
void BLHelis::Actuate_Motor_4(void){
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_4, MOTOR_IDLE);
}

void BLHelis::Start(void){
	static uint16_t mc = MOTOR_1MS;
	motor_sp msp = {mc, mc, mc, mc};
	this->Update_Motor_SP(msp);
	vTaskDelay(2000);

}

void BLHelis::Update_Motor_SP(motor_sp msp){
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_1, msp.m1_sp);
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_2, msp.m2_sp);
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_3, msp.m3_sp);
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_4, msp.m4_sp);
}

void BLHelis::Set_Thrust_Percent(float *percent){
	uint16_t mc = MOTOR_1MS + (int)((MOTOR_2MS - MOTOR_1MS) * *percent);
	motor_sp msp;
	msp.m1_sp = mc;
	msp.m2_sp = mc;
	msp.m3_sp = mc;
	msp.m4_sp = mc;

	this->Update_Motor_SP(msp);
}
}
