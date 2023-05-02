//
// Created by PC-HOME on 02.05.2023.
//

#ifndef BLDC_CONTROL_STATE_MACHINE_H
#define BLDC_CONTROL_STATE_MACHINE_H

/// \typedef enum states
/// \brief Перечисление для состояний конечного автомата
typedef enum {
    bldc_control_before_calibration, ///<Система не откалибрована
    bldc_control_found_zero, ///<Найдена нулевая точка (Касание датчика A)
    bldc_control_found_end, ///<Найдена конечная точка (Касание датчика B). Калибровка окончена
    bldc_control_setup, ///<Состояние установки желаемого положения
    bldc_control_positioning ///< Состояние осуществления позиционирования
}states;

#endif //BLDC_CONTROL_STATE_MACHINE_H
