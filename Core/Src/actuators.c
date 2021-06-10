/*
 * actuators.c
 *
 *  Created on: Jun 9, 2021
 *      Author: danie
 */

#include "actuators.h"

void Init_Motor_PWM(void){
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void Set_Motor_PWM(uint16_t* m_compare_1, uint16_t* m_compare_2, uint16_t* m_compare_3, uint16_t* m_compare_4){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, *m_compare_1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, *m_compare_2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, *m_compare_3);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, *m_compare_4);
}
