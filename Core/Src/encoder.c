#include <encoder.h>


uint32_t get_encoder_value(TIM_TypeDef* encoder_TIM) {
	return encoder_TIM->CNT;
}

/// \brief Рассчёт скорости вращения энкодера
///
/// \param[in] encoder_TIM - Указатель на структуру таймера, к которому подключен энеодер
/// \param[in] prev_ticks - Предыдущее значение счётчика
/// \param[in] encoder_resolution - Разрешение энкодера, имп./об.
/// \param[in] samp_freq - Частота опроса энкодера
/// \return скорость вращения, rpm
double get_speed_rpm(TIM_HandleTypeDef *encoder_TIM, uint16_t prev_ticks, uint16_t encoder_resolution, uint16_t samp_freq) {
//        optical_count = __HAL_TIM_GET_COUNTER(&htim3); /// 0...65535
//        /// Рассчитываем разницу между соседними измерениями тиков энкодера
//        int32_t ticks_div = (int32_t) optical_count -  (int32_t) prev_pos_ticks;
//        int32_t speed_ticks =  ticks_div *  sampling_frequency; /// Скорость, тиков / секунду
//        speed_rpm = ((double) speed_ticks /  BLDC_CONTROL_ENCODER_RES) * 60.; ///Скорость, об/мин
    int32_t ticks_div = (int32_t) __HAL_TIM_GET_COUNTER(encoder_TIM) -  (int32_t) prev_ticks;
    int32_t speed_ticks =  ticks_div *  samp_freq; /// Скорость, тиков / секунду
    double speed_rpm = ((double) speed_ticks / (double) encoder_resolution) * 60.; ///Скорость, об/мин
    return speed_rpm;
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
