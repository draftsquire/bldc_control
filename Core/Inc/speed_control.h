#ifndef INC_SPEED_CONTROL_H_
#define INC_SPEED_CONTROL_H_

#include "main.h"
/// \def BLDC_CONTROL_MAX_CONTROL
/// \brief Максимальное управляющее воздействие для установки скорости
#define BLDC_CONTROL_MAX_CONTROL 500

/// \brief Установка скорости вращения движка в процентах
///
/// \param speed[in] - Величина скорости в условных единицах [-500..500]
/// \param pwm_tim[in] - Хэндлер таймера, настроенного на генерацию ШИМ
void set_speed(int16_t speed, TIM_HandleTypeDef *pwm_tim);

/// \brief Конвертация координат в мм в количество отчётов энкодера
///
/// \param guide_len[in] - Фактическая длина направляющей (рабочей зоны)
/// \param max_ticks[in] - Максимальное количество отсчётов энкодера. Соответствует крайней точке перемещения, то есть концевому датчику B
/// \param coord_mm[in] - Координата в мм
/// \return Количество отсчётов энкодера
uint16_t get_ticks_from_mm(int16_t guide_len, uint16_t max_ticks, double coord_mm);

/// \brief Конвертация отсчётов энкодера в координату в миллиметрах
///
/// \param guide_len[in] - Фактическая длина направляющей (рабочей зоны)
/// \param max_ticks[in] - Максимальное количество отсчётов энкодера. Соответствует крайней точке перемещения, то есть концевому датчику B
/// \param ticks[in] - Количество отсчётов энкодера
/// \return Координата в миллиметрах
double get_mm_from_ticks(int16_t guide_len, uint16_t max_ticks, uint16_t ticks);

#endif /* INC_SPEED_CONTROL_H_ */
