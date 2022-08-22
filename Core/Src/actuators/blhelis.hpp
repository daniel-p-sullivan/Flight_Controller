/*
 * blhelis.hpp
 *
 *  Created on: Aug 21, 2022
 *      Author: danie
 */

#ifndef SRC_BLHELIS_HPP_
#define SRC_BLHELIS_HPP_

#include "actuators.hpp"

#define MOTOR_IDLE 3214  //can be optimized more
#define MOTOR_IDLE_2
#define MOTOR_1MS 3120
#define MOTOR_2MS 6242
#define MOTOR_15MS 4846
#define MOTOR_13MS 4200

namespace actuators{

class BLHelis : Motors{
public:
	BLHelis(TIM_HandleTypeDef htim8);
	void Update_Motor_SP(motor_sp& setpoint);
private:
	TIM_HandleTypeDef* timer;
	void Init_Motors(void);
	void Start(void);
	void Set_Thrust_Percent(float *percent);
	void Actuate_Motor_1(void);
	void Actuate_Motor_2(void);
	void Actuate_Motor_3(void);
	void Actuate_Motor_4(void);
};

}

#endif