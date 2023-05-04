#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

/// \brief Рассчёт скорости вращения энкодера
///
/// \param[in] encoder_TIM - Указатель на структуру таймера, к которому подключен энеодер
/// \param[in] encoder_resolution - Разрешение энкодера, имп./об.
/// \param[in] samp_freq - Частота опроса энкодера
/// \return скорость вращения, rpm
double
get_speed_rpm_freq(TIM_HandleTypeDef *encoder_TIM, uint16_t prev_ticks, uint16_t curr_ticks, uint16_t encoder_resolution,
                   uint16_t samp_freq);
/// \brief Рассчёт скорости вращения энкодера. Метод, основанный на измерении периода
///
/// \param[in] n - Количество высокочастотных импульсов за период следования импульсов энкодера
/// \param[in] encoder_resolution - Разрешение энкодера, имп./об.
/// \param[in] samp_freq - Частота следования ВЧ импульсов
/// \return скорость вращения, rpm
double get_speed_rpm_period(uint16_t n, uint16_t encoder_resolution, uint32_t samp_freq);

uint32_t get_encoder_value(TIM_TypeDef* encoder_TIM);
#endif /* INC_ENCODER_H_ */
