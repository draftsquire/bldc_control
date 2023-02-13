#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

int16_t get_encoder_value(TIM_TypeDef* encoder_TIM);
float getMotorAngleDeg(int16_t encoder_counter, int16_t ticks_per_revolution);
float getMotorAngleRad(int16_t encoder_counter, int16_t ticks_per_revolution);
float getDriveAngle(float motor_angle, float gearbox_ratio);
float getLinearDistance(TIM_TypeDef* encoder_TIM, int16_t ticks_per_revolution, float gearbox_ratio, float pulley_diameter);

#endif /* INC_ENCODER_H_ */
