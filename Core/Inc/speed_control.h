#ifndef INC_SPEED_CONTROL_H_
#define INC_SPEED_CONTROL_H_

#include "main.h"

///// \brief Установка скорости вращения движка в процентах
/////
///// \param speed[in] - Величина скорости в условных единицах [-500..500]
///// \param pwm_tim[in] - Хэндлер таймера, настроенного на генерацию ШИМ
void set_speed(int16_t speed, TIM_HandleTypeDef *pwm_tim);


#endif /* INC_SPEED_CONTROL_H_ */
