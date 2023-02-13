#include "speed_control.h"

int16_t Limit_and_get_motor_control_timer_value(int16_t lower_limit, int16_t upper_limit, TIM_HandleTypeDef* htim3) {
	int16_t count = __HAL_TIM_GET_COUNTER(htim3);
	if (count < 0) {
		TIM3 -> CNT = 0;
	} else if (count > 100) {
		TIM3 -> CNT = 100;
	}
	return count;
}

int16_t Get_PWM_width_from_speed_value(
		int16_t lower_pwm_width_limit,
		int16_t upper_pwm_width_limit,
		int16_t lower_speed_value_limit,
		int16_t upper_speed_value_limit,
		int16_t speed_value) {
	return (int16_t)
			(lower_pwm_width_limit +
			(upper_pwm_width_limit - lower_pwm_width_limit) /
			(upper_speed_value_limit - lower_speed_value_limit) *
			speed_value);
}
