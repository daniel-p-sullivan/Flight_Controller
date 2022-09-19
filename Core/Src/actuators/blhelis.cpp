/*
 * blhelis.cpp
 *
 *  Created on: Aug 21, 2022
 *      Author: danie
 */


#include "blhelis.hpp"

namespace actuators{

BLHelis::BLHelis(TIM_HandleTypeDef& rhtim8) : timer(rhtim8){}


void BLHelis::initMotors(void){
	HAL_TIM_Base_Start(&(this->timer));
	HAL_TIM_PWM_Start(&(this->timer), TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&(this->timer), TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&(this->timer), TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&(this->timer), TIM_CHANNEL_4);
	static uint16_t mc1_l = MOTOR_1MS;
	static uint16_t mc1_h = MOTOR_13MS;
	static uint16_t mc1 = 0;
	static uint16_t step_count = 1000;
	static uint8_t delay = 1;
	float step = ((float)mc1_h - (float)mc1_l) / (float)step_count;

	motor_sp msp;

	for(float i = 0; i < (step_count); i++){
		mc1 = mc1_l + (int)(i * step);
		msp = {mc1, mc1, mc1, mc1};
		this->Update_Motor_SP(msp);
		vTaskDelay(delay);
	}
	for(float i = 0; i < (step_count); i++){
		mc1 = mc1_h - (int)(i * step);
		msp = {mc1, mc1, mc1, mc1};
		this->Update_Motor_SP(msp);
		vTaskDelay(delay);
	}
	mc1 = mc1_l;
	msp = {mc1, mc1, mc1, mc1};
	this->Update_Motor_SP(msp);
	vTaskDelay(10000);
}

void BLHelis::actuateMotors(state::QuadControlActions& ac){

	static uint16_t m1_sp, m2_sp, m3_sp, m4_sp;

	//mix the controller output
	m1_sp = (uint16_t)std::max((int16_t)MOTOR_BASEMS, (int16_t)((float)MOTOR_BASEMS + ac.u1 - ac.u2 - ac.u3 + ac.u4));
	m2_sp = (uint16_t)std::max((int16_t)MOTOR_BASEMS, (int16_t)((float)MOTOR_BASEMS + ac.u1 + ac.u2 + ac.u3 + ac.u4));
	m3_sp = (uint16_t)std::max((int16_t)MOTOR_BASEMS, (int16_t)((float)MOTOR_BASEMS + ac.u1 - ac.u2 + ac.u3 - ac.u4));
	m4_sp = (uint16_t)std::max((int16_t)MOTOR_BASEMS, (int16_t)((float)MOTOR_BASEMS + ac.u1 + ac.u2 - ac.u3 - ac.u4));

	//ensure that the motors are not too fast
	m1_sp = std::min(m1_sp, (uint16_t)MOTOR_175MS);
	m2_sp = std::min(m2_sp, (uint16_t)MOTOR_175MS);
	m3_sp = std::min(m3_sp, (uint16_t)MOTOR_175MS);
	m4_sp = std::min(m4_sp, (uint16_t)MOTOR_175MS);

	if(m1_sp == MOTOR_175MS || m2_sp == MOTOR_175MS || m3_sp == MOTOR_175MS || m4_sp == MOTOR_175MS){
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);  //show that you're maxing out
	}

	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_1, (uint16_t)m1_sp);
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_2, (uint16_t)m2_sp);
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_3, (uint16_t)m3_sp);
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_4, (uint16_t)m4_sp);
}


void BLHelis::Actuate_Motor_1(void){
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_1, MOTOR_11MS);
}

void BLHelis::Actuate_Motor_2(void){
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_2, MOTOR_11MS);
}
void BLHelis::Actuate_Motor_3(void){
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_3, MOTOR_11MS);
}
void BLHelis::Actuate_Motor_4(void){
	__HAL_TIM_SET_COMPARE(&(this->timer), TIM_CHANNEL_4, MOTOR_11MS);
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
