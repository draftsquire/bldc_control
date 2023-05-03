//
// Created by PC-HOME on 02.05.2023.
//

#ifndef BLDC_CONTROL_STATE_MACHINE_H
#define BLDC_CONTROL_STATE_MACHINE_H

/// \typedef enum states
/// \brief Перечисление для состояний конечного автомата
typedef enum {
    bldc_control_before_calibration, ///<Система не откалибрована
    bldc_control_calibrating_zero, ///< Калибруется нулевая точка
    bldc_control_calibrated_zero, /// < Нулевая точка найдена
    bldc_control_calibrating_end, ///< Калибруется конечная точка
    bldc_control_calibrated_end, ///<Найдена конечная точка (Касание датчика B). Калибровка окончена
    bldc_control_calibrated, ///<Состояние успешной калибровки
    bldc_control_ready_4_positioning,  ///< Состояние готовности к получению задания
    bldc_control_positioning, ///< Состояние осуществления позиционирования
    bldc_control_collision_error ///< коллизия
}states;

#endif //BLDC_CONTROL_STATE_MACHINE_H
