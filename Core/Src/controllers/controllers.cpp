/*
 * controllers.c
 *
 *  Created on: Jun 9, 2021
 *      Author: danie
 */

#include "controllers.hpp"

#include "../state/state.hpp"
#include "../state/trajectories.hpp"
#include "../model/models.hpp"

namespace control{

PI::PI(float sp, float dt, float Kp, float Ki) :
		setpoint(sp), dt(dt), Kp(Kp), Ki(Ki), errorIntegrator(dt){}

float PI::calcOutput(float stateEstimate){
		static float error = 0;
		static float ierror = 0;

		error = setpoint - stateEstimate;
		ierror = errorIntegrator.integrate(error);

		if(ierror > INTEGRAL_WINDUP_MAX){
			ierror = INTEGRAL_WINDUP_MAX;
		}
		else if(ierror < INTEGRAL_WINDUP_MIN){
			ierror = INTEGRAL_WINDUP_MIN;
		}
		return (Kp * error + Ki * ierror);
}


PID::PID(float sp, float dt, float Kp, float Ki, float Kd) :
		setpoint(sp), dt(dt), Kp(Kp), Ki(Ki), Kd(Kd) ,errorIntegrator(dt), errorDifferentiator(dt){}

float PID::calcOutput(float stateEstimate){
		static float error = 0;
		static float ierror = 0;
		static float derror = 0;

		error = setpoint - stateEstimate;
		ierror = errorIntegrator.integrate(error);
		derror = errorDifferentiator.differentiate(error);

		if(ierror > INTEGRAL_WINDUP_MAX){
			ierror = INTEGRAL_WINDUP_MAX;
		}
		else if(ierror < INTEGRAL_WINDUP_MIN){
			ierror = INTEGRAL_WINDUP_MIN;
		}
		return (Kp * error + Ki * ierror + Kd * derror);
}

}



























//void Iter_PI(PI *self){
//
//	self->error = self->sp - self->meas;
//	self->integral_error += self->error * self->dt;
//	self->output = Kp * self->error + Ki * self->integral_error;
//
//}
//
//PI* Construct_PI(uint8_t dt){
//	PI *self = malloc(sizeof(PI));
//	self->dt = dt;
//	return self;
//}
//
//void Update_Sample(TRPY_Controller* self, IMU_Sample* sample){
//	self->roll_PI->meas = sample->g_x;
//	self->pitch_PI->meas = sample->g_y;
//	self->yaw_PI->meas = sample->g_z;
//	self->thrust_PI->meas = sample->a_z;
//}
//void Update_Setpoint(TRPY_Controller* self, TRPY_Setpoint* trpy_sp){
//	self->roll_PI->sp = trpy_sp->roll_setpoint;
//	self->pitch_PI->sp = trpy_sp->pitch_setpoint;
//	self->yaw_PI->sp = trpy_sp->yaw_setpoint;
//	self->thrust_PI->sp = trpy_sp->z_setpoint;
//}
//
//TRPY_Controller* Construct_TRPY_Controller(int8_t dt){
//
//	static PI* roll_PI, *pitch_PI, *yaw_PI, *thrust_PI;
//	TRPY_Controller* self = malloc(sizeof(TRPY_Controller));
//
//	roll_PI = Construct_PI(dt);
//	pitch_PI = Construct_PI(dt);
//	yaw_PI = Construct_PI(dt);
//	thrust_PI = Construct_PI(dt);
//
//	self->roll_PI = roll_PI;
//	self->pitch_PI = pitch_PI;
//	self->yaw_PI = yaw_PI;
//	self->thrust_PI = thrust_PI;
//
//	self->trpy_o = malloc(sizeof(TRPY_Output));
//
//	return self;
//
//}
//
//void Iter_TRPY_Controller(TRPY_Controller* self, IMU_Sample *sample){
//
//	Update_Sample(self, sample);
//
//	Iter_PI(self->roll_PI);
//	Iter_PI(self->pitch_PI);
//	Iter_PI(self->yaw_PI);
//	Iter_PI(self->thrust_PI);
//
//	self->trpy_o->thrust_output = self->thrust_PI->output;
//	self->trpy_o->roll_output = self->roll_PI->output;
//	self->trpy_o->pitch_output = self->pitch_PI->output;
//	self->trpy_o->yaw_output = self->yaw_PI->output;
//
//}
//
//void Motor_Mixing_Algorithm(TRPY_Output *trpy_o){
//
//	static uint16_t m_compare_1, m_compare_2, m_compare_3, m_compare_4 = 0;
//	static int16_t base;
//	base = MOTOR_IDLE + trpy_o->thrust_output;
//
//	m_compare_1 = pos(base + trpy_o->roll_output + trpy_o->pitch_output + trpy_o->yaw_output);
//	m_compare_2 = pos(base - trpy_o->roll_output + trpy_o->pitch_output - trpy_o->yaw_output);
//	m_compare_4 = pos(base + trpy_o->roll_output - trpy_o->pitch_output - trpy_o->yaw_output);
//	m_compare_3 = pos(base - trpy_o->roll_output - trpy_o->pitch_output + trpy_o->yaw_output);
//
//	Set_Motor_PWM(&m_compare_1, &m_compare_2, &m_compare_3, &m_compare_4);
//
//}
//
//inline uint16_t pos(int16_t x){
//	if(x >= 0){
//		return x;
//	}else{
//		return 0;
//	}
//}
