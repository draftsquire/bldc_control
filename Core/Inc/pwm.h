#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "main.h"
#include <math.h>

void pwm_custom_init(TIM_TypeDef *pwm_tim, uint32_t freq);
int pwm_change_duty(TIM_TypeDef *pwm_tim, uint8_t duty_cycle);
void PWM_generate_from_buffer(uint8_t* requested_pulse_byte_buffer);
uint32_t Buffer_to_uint32_t(uint8_t* buffer);

#endif /* INC_PWM_H_ */
