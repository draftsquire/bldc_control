/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>



#include <encoder.h>

#include <state_machine.h>
#include <speed_control.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_F 10000

/// \def BLDC_CONTROL_SET_DIR_FWD
/// \brief Макрос для установки направления движения двигателя вперёд (в направлении ОТ двигателя по рельсе).
/// \warning �?спользовать с осторожностью, т.к. подключение пина, отвечающего за направление вращения двигателя, может измениться
#define BLDC_CONTROL_SET_DIR_FWD() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)

/// \def BLDC_CONTROL_SET_DIR_BWD
/// \brief Макрос для установки направления движения двигателя назад (в направлении К двигателю по рельсе).
/// \warning �?спользовать с осторожностью, т.к. подключение пина, отвечающего за направление вращения двигателя, может измениться
#define BLDC_CONTROL_SET_DIR_BWD() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)

/// \def BLDC_CONTROL_INTERNAL_CLK_FREQ
/// \brief Частота тактирования контроллера, Гц
#define BLDC_CONTROL_INTERNAL_CLK_FREQ 16000000

/// \def BLDC_CONTROL_ENCODER_RES
/// \brief Разрешение измерительного энкодера, имп./об.
#define BLDC_CONTROL_ENCODER_RES 64


/// \def BLDC_CONTROL_CART_LEN
/// \brief Длина каретки
#define BLDC_CONTROL_CART_LEN 45

/// \def BLDC_CONTROL_GUIDE_LEN
/// \brief Фактическая длина рабочей зоны в мм
#define BLDC_CONTROL_GUIDE_LEN (250 - BLDC_CONTROL_CART_LEN)



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
char usart_buffer[255] = {'\0'};
uint8_t recieve_buffer[255] = {'\0'};
char transmit_buffer[255] = {'\0'};
int16_t control_speed = 0;
int16_t position_ticks = 0;

double speed_rpm = 0.;

double position_mm = 0.;
uint16_t max_position_ticks = 0;

uint16_t desired_position_ticks = 0;
double desired_position_mm = 0;
double ticks_per_mm = 0.;
double p_coeff = 1;
double d_coeff = 1;
control_type control = bldc_control_proportional;
int16_t positioning_error_ticks = 0;
uint16_t encoder_ic = 0;

uint16_t sampling_frequency;
uint8_t direct;
states state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if ((htim->Instance == TIM4) && ((state == bldc_control_before_calibration) || (state == bldc_control_calibrated))) {
        /// Считывание показаний механического энкодера
        control_speed = (int16_t) (10 * __HAL_TIM_GET_COUNTER(&htim1));
        if (control_speed > BLDC_CONTROL_MAX_CONTROL){
            control_speed = BLDC_CONTROL_MAX_CONTROL;
            __HAL_TIM_SET_COUNTER(&htim1, 50);
        }else if (control_speed < - BLDC_CONTROL_MAX_CONTROL){
            control_speed = - BLDC_CONTROL_MAX_CONTROL;
            __HAL_TIM_SET_COUNTER(&htim1, UINT16_MAX - 50);
        }
        /// speed  [-500..500]
        set_speed(control_speed, &htim2);
    }
    /// С частотой регулирующего Ш�?М
    if ((htim == &htim2) && (state == bldc_control_positioning) ){
        position_ticks =  (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        positioning_error_ticks =  (int16_t)(desired_position_ticks - position_ticks);
        if (control == bldc_control_proportional){
            control_speed = (int16_t) round(p_coeff * positioning_error_ticks);
        }else if ( control == bldc_control_prop_diff){
            control_speed = 0;
        }
        if (control_speed > 500){
            control_speed = 500;
        }else if (control_speed < -500){
            control_speed = -500;
        }
        set_speed(control_speed, &htim2);

    }
    if (htim->Instance == TIM5) {
        switch (state){
            case bldc_control_before_calibration:
//                snprintf(transmit_buffer, sizeof(transmit_buffer), "before_calibration. PWM: %lu , Optical encoder: %ld\n Speed: %f rpm\n\r",TIM2->CCR1, __HAL_TIM_GET_COUNTER(&htim3),  speed_rpm);
                snprintf(transmit_buffer, sizeof(transmit_buffer), "before_calibration. PWM: %lu ,speed: %d\n\r", TIM2->CCR1, control_speed);

                break;
            case bldc_control_calibrating_zero:
                snprintf(transmit_buffer, sizeof(transmit_buffer), "calibrating_zero. Optical encoder: %hd\n Speed: %f rpm\n\r", (int16_t) __HAL_TIM_GET_COUNTER(&htim3),  speed_rpm);
                break;
            case bldc_control_calibrated_zero:
                snprintf(transmit_buffer, sizeof(transmit_buffer), "calibrated_zero. Optical encoder: %hd\n Speed: %f rpm\n\r",(int16_t)  __HAL_TIM_GET_COUNTER(&htim3),  speed_rpm);
                break;
            case bldc_control_calibrating_end:
                snprintf(transmit_buffer, sizeof(transmit_buffer), "calibrating_end. Optical encoder: %hd\n Speed: %f rpm\n\r",(int16_t)  __HAL_TIM_GET_COUNTER(&htim3),  speed_rpm);
                break;
            case bldc_control_calibrated_end:
                snprintf(transmit_buffer, sizeof(transmit_buffer), "calibrated_end. Optical encoder: %hd\n Speed: %f rpm\n\r",(int16_t)  __HAL_TIM_GET_COUNTER(&htim3), speed_rpm);
                break;
            case bldc_control_calibrated:
                snprintf(transmit_buffer, sizeof(transmit_buffer), "Calibrated!!!. Optical encoder: %hd\n Speed: %f rpm\n\r",(int16_t)  __HAL_TIM_GET_COUNTER(&htim3), speed_rpm);
                break;
            case bldc_control_ready_4_positioning:
                position_mm =(double) __HAL_TIM_GET_COUNTER(&htim3) * (double) BLDC_CONTROL_GUIDE_LEN / (double)max_position_ticks;
                snprintf(transmit_buffer, sizeof(transmit_buffer), "Ready for positioning. Position, mm: %f\n Speed: %f rpm\n\r", position_mm, speed_rpm);
                break;
            default:
                snprintf(transmit_buffer, sizeof(transmit_buffer), "Coordinates: %0.2f, Optical encoder: %ld\n Control speed: %d rpm\n\r",get_mm_from_ticks(BLDC_CONTROL_GUIDE_LEN, max_position_ticks, __HAL_TIM_GET_COUNTER(&htim3)),
                         __HAL_TIM_GET_COUNTER(&htim3), control_speed);
                break;
        }

        HAL_UART_Transmit_DMA(&huart2, (uint8_t *) transmit_buffer, strlen(transmit_buffer));

    }
}

/// \brief Callback для внешних прерываний с концевых датчиков
/// \note Прерывания на ножках PC9, PC8 генерируются по спадающему фронту
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    /// Сработал концевой датчик B (дальше от движка)
    if (GPIO_Pin == TS_B_IN_Pin) {
        /// "Сброс" счётчика механического энкодера
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        /// Тормоз двигателя
        set_speed(0, &htim2);

        if (state == bldc_control_calibrating_end){
            max_position_ticks = __HAL_TIM_GET_COUNTER(&htim3);
            ticks_per_mm =  round((double)max_position_ticks / (double) BLDC_CONTROL_GUIDE_LEN);
            state = bldc_control_calibrated_end;
        }else if (state == bldc_control_positioning){
            state = bldc_control_calibrated;
        }else {
            state = bldc_control_collision_error;
        }
    /// Сработал концевой датчик A (Ближе к движку)
    } else if (GPIO_Pin == TS_A_IN_Pin) {
        /// "Сброс" счётчика механического энкодера
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        /// Тормоз двигателя
        set_speed(0, &htim2);
        if(state == bldc_control_calibrating_zero){
            state = bldc_control_calibrated_zero;
        }else if (state == bldc_control_positioning){
            state = bldc_control_calibrated;
        }else {
            state = bldc_control_collision_error;
        }
    } else if (GPIO_Pin == EN_BUTTON_IN_Pin){
        if(state == bldc_control_before_calibration){
            state = bldc_control_calibrating_zero;
        }else if (state == bldc_control_calibrated){
            state = bldc_control_ready_4_positioning;
        }
    }

}

/// \brief Callback прерывания по захвату переднего фронта сигнала
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    /// Ножка, подключенная к одному из каналов энкодера
    if(htim->Instance == TIM10){
        encoder_ic = __HAL_TIM_GET_COMPARE(&htim10, TIM_CHANNEL_1);    //read TIM10 channel 1 capture value
        __HAL_TIM_SET_COUNTER(&htim10, 0);    //reset counter after input capture interrupt occurs
        speed_rpm = get_speed_rpm_period(encoder_ic, BLDC_CONTROL_ENCODER_RES, 100000);
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim9);
  HAL_TIM_IC_Start_IT(&htim10, TIM_CHANNEL_1);

  set_speed(0, &htim2);
  sampling_frequency = BLDC_CONTROL_INTERNAL_CLK_FREQ / (TIM9->PSC + 1) / (TIM9->ARR + 1);
//  HAL_UART_Receive_DMA(&huart2, (uint8_t *) recieve_buffer, 3);
  /// < Устанавливаем начальное состояние системы
  state = bldc_control_before_calibration;

  double error = 0.;

  while (1){

      switch (state) {
          case bldc_control_calibrating_zero :
              __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 600); /// ~ минимальная скорость
              BLDC_CONTROL_SET_DIR_BWD();
//              set_speed(- BLDC_CONTROL_MAX_CONTROL / 5, &htim2);
              break;
          case bldc_control_calibrated_zero:
              /// Ожидание окончание вращения энкодера по инерции после столкновения с датчиком
              set_speed(0, &htim2);
              HAL_Delay(1500);
              /// Обнуление счётчика оптического энкодера
              __HAL_TIM_SET_COUNTER(&htim3, 0);
              state = bldc_control_calibrating_end;
              break;
          case bldc_control_calibrating_end:
              if ( (__HAL_TIM_GET_COUNTER(&htim3) < 1400) && (__HAL_TIM_GET_COUNTER(&htim3) >= 0) ){
                  set_speed(BLDC_CONTROL_MAX_CONTROL, &htim2);
              } else{
                  set_speed(BLDC_CONTROL_MAX_CONTROL / 5, &htim2);
              }
              break;
          case bldc_control_calibrated_end:
              if ( (__HAL_TIM_GET_COUNTER(&htim3) >= 800) || (__HAL_TIM_GET_COUNTER(&htim3) <= 500)){
                  set_speed(- BLDC_CONTROL_MAX_CONTROL / 2, &htim2);
              }else { /// Отъехали от края на безопасное расстояние, можно останавливаться и готовиться к работе
                  set_speed(0, &htim2);
                  state = bldc_control_calibrated;
              }
              break;
          case bldc_control_ready_4_positioning:
              set_speed(0, &htim2);
              desired_position_ticks = get_ticks_from_mm(BLDC_CONTROL_GUIDE_LEN, max_position_ticks, desired_position_mm);
              snprintf(transmit_buffer, sizeof(transmit_buffer), "Desired position: %0.1f mm, %d ticks\n\r",desired_position_mm, desired_position_ticks);
//              HAL_Delay(1000);
              HAL_UART_Transmit_DMA(&huart2, (uint8_t *) transmit_buffer, strlen(transmit_buffer));
              state = bldc_control_positioning;
              break;
//          case bldc_control_positioning:
//
//              break;
          default:
              break;


      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 15999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 15999;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 49;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 159;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim10, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 256000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MOTOR_DIR_Pin */
  GPIO_InitStruct.Pin = MOTOR_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TS_A_IN_Pin TS_B_IN_Pin */
  GPIO_InitStruct.Pin = TS_A_IN_Pin|TS_B_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_BUTTON_IN_Pin */
  GPIO_InitStruct.Pin = EN_BUTTON_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EN_BUTTON_IN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
