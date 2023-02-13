#include <pwm.h>

/// \def F_CLCK
/// \brief частота тактирования таймера до делителя
#define F_CLCK  16000000

float time_coeff;

/// \brief Функция для настройки таймера под ШИМ определённой частоты
///
/// \param[in] pwm_tim - Указатель на структуру HAL таймера
/// \param[in] freq желаемая частота в Гц,   не менее 250Гц, не более F_CLCK / 10 Гц
void pwm_custom_init(TIM_TypeDef *pwm_tim, uint32_t freq) {
	uint16_t PSC_value;
    uint16_t ARR_value;
//	PSC_value = (uint32_t)ceil(((double)F_CLCK / ((65535.0+1.0)*50.0)) - 1.0);
    uint16_t freq_ratio = F_CLCK / freq;
    if(freq_ratio < 10){
        ARR_value = 0;
    }else if (freq_ratio < 100){
        ARR_value = 10 - 1;
    }else if (freq_ratio < 1000){
        ARR_value = 100 - 1;
    }else{
        ARR_value = 1000 - 1;
    }
    PSC_value = (uint16_t) ( F_CLCK/ ((ARR_value + 1) * freq) - 1);
    pwm_tim -> PSC = PSC_value;
    pwm_tim -> ARR = ARR_value;
}

/// \brief Функция для установки скважности работающего ШИМ сигнала
///
/// \param[in] pwm_tim - Указатель на структуру HAL таймера
/// \param[in] duty_cycle - Скважность, [0;100]%
/// \return Код выхода. 0 - ОК, -1 - Скважность задана некорректно.
int pwm_change_duty(TIM_TypeDef *pwm_tim, uint8_t duty_cycle) {
    if(duty_cycle>100){
        return -1;
    }
    uint16_t CCR_value = pwm_tim->ARR * ((double) duty_cycle / 100);
    pwm_tim-> CCR1 = CCR_value; //(CNT <CCR1) -> GENERATE PWM PULSE
    return 1;
}


