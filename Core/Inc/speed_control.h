#ifndef INC_SPEED_CONTROL_H_
#define INC_SPEED_CONTROL_H_

#include "main.h"

int16_t Limit_and_get_motor_control_timer_value(int16_t lower_limit, int16_t upper_limit, TIM_HandleTypeDef* htim3);

int16_t Get_PWM_width_from_speed_value(
		int16_t lower_pwm_width,
		int16_t upper_pwm_width,
		int16_t lower_speed_value_limit,
		int16_t upper_speed_value_limit,
		int16_t speed_value);

#endif /* INC_SPEED_CONTROL_H_ */
