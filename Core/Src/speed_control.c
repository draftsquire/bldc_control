#include "speed_control.h"
#include "math.h"
///// \brief Установка скорости вращения движка в процентах
/////
///// \param speed[in] - Величина скорости в условных единицах [-500..500]
///// \param pwm_tim[in] - Хэндлер таймера, настроенного на генерацию ШИМ
void set_speed(int16_t speed, TIM_HandleTypeDef *pwm_tim) {
    if (speed == 0) {
        __HAL_TIM_SET_COMPARE(pwm_tim, TIM_CHANNEL_1, 0);
    } else if (speed < 0) {
        __HAL_TIM_SET_COMPARE(pwm_tim, TIM_CHANNEL_1, (-speed)  + BLDC_CONTROL_MAX_CONTROL - 1); // длина импульса
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    } else {
        __HAL_TIM_SET_COMPARE(pwm_tim, TIM_CHANNEL_1, speed + BLDC_CONTROL_MAX_CONTROL - 1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    }
}

/// \brief Конвертация координат в мм в количество отчётов энкодера
///
/// \param guide_len[in] - Фактическая длина направляющей (рабочей зоны)
/// \param max_ticks[in] - Максимальное количество отсчётов энкодера. Соответствует крайней точке перемещения, то есть концевому датчику B
/// \param coord_mm[in] - Координата в мм
/// \return Количество отсчётов энкодера
uint16_t get_ticks_from_mm(int16_t guide_len, uint16_t max_ticks, double coord_mm){
    uint16_t ticks;
    double ticks_per_mm = (double) max_ticks / guide_len;
    ticks = (uint16_t) round(coord_mm * ticks_per_mm);
    return ticks;
}

/// \brief Конвертация отсчётов энкодера в координату в миллиметрах
///
/// \param guide_len[in] - Фактическая длина направляющей (рабочей зоны)
/// \param max_ticks[in] - Максимальное количество отсчётов энкодера. Соответствует крайней точке перемещения, то есть концевому датчику B
/// \param ticks[in] - Количество отсчётов энкодера
/// \return Координата в миллиметрах
double get_mm_from_ticks(int16_t guide_len, uint16_t max_ticks, uint16_t ticks){
    double coord_mm;
    double mm_per_tick = (double)guide_len / max_ticks;
    coord_mm = mm_per_tick * (double) ticks;
    return coord_mm;
}


