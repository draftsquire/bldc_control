#include "speed_control.h"

///// \brief Установка скорости вращения движка в процентах
/////
///// \param speed[in] - Величина скорости в условных единицах [-500..500]
///// \param pwm_tim[in] - Хэндлер таймера, настроенного на генерацию ШИМ
void set_speed(int16_t speed, TIM_HandleTypeDef *pwm_tim) {

    if (speed == 0) {
        __HAL_TIM_SET_COMPARE(pwm_tim, TIM_CHANNEL_1, 0);
    } else if (speed < 0) {
        __HAL_TIM_SET_COMPARE(pwm_tim, TIM_CHANNEL_1, (-speed) - 1); // длина импульса
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    } else {
        __HAL_TIM_SET_COMPARE(pwm_tim, TIM_CHANNEL_1, (speed) - 1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    }

}

