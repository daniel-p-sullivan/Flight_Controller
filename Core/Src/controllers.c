/*
 * controllers.c
 *
 *  Created on: Jun 9, 2021
 *      Author: danie
 */

#include "controllers.h"

int32_t PI(int16_t* sp, int16_t *meas, int16_t *i_error){


	int16_t error = *sp - *meas;
	int16_t integrated_error = *i_error + (*sp - *meas) * dt_control_loop;

	return Kp * error + Ki * integrated_error;

}

void trpy_Controller(Setpoint *sp, IMU_sample *sample){

	static int16_t roll, pitch, yaw, thrust = 0;
	int16_t temp = 0;
	roll = PI(sp->roll_setpoint, sample->g_x, &temp);

}

void Motor_Mixing_Algorithm(int16_t *thrust, int16_t *roll, int16_t *pitch, int16_t *yaw){

	static int16_t m_compare_1, m_compare_2, m_compare_3, m_compare_4 = 0;
	static uint16_t base;
	base = MOTOR_IDLE + *thrust;

	m_compare_1 = base + *roll + *pitch + *yaw;
	m_compare_2 = base - *roll + *pitch - *yaw;
	m_compare_3 = base + *roll - *pitch - *yaw;
	m_compare_4 = base - *roll - *pitch + *yaw;

	Set_Motor_PWM(&m_compare_1, &m_compare_2, &m_compare_3, &m_compare_4);

}
