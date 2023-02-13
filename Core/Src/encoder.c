#include <encoder.h>

int16_t get_encoder_value(TIM_TypeDef* encoder_TIM) {
	return encoder_TIM->CNT;
}

float getMotorAngleRad(int16_t encoder_counter, int16_t ticks_per_revolution) {
	return 2 * 3.14159 * ((float) encoder_counter / (float)ticks_per_revolution);
}
float getMotorAngleDeg(int16_t encoder_counter, int16_t ticks_per_revolution) {
	return 360 * ((float) encoder_counter / (float)ticks_per_revolution);
}

float getDriveAngle(float motor_angle, float gearbox_ratio) {
	return motor_angle * gearbox_ratio;
}

float getLinearDistance(TIM_TypeDef* encoder_TIM, int16_t ticks_per_revolution, float gearbox_ratio, float pulley_diameter) {
	int16_t encoder_counter = get_encoder_value(encoder_TIM);
	float motor_angle = getMotorAngleDeg(encoder_counter, ticks_per_revolution);
	float drive_angle = getDriveAngle(motor_angle, gearbox_ratio);
	return drive_angle * pulley_diameter;
}
