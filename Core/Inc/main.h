/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Relay_1_Pin GPIO_PIN_7
#define Relay_1_GPIO_Port GPIOB
#define Input_1_Pin GPIO_PIN_0
#define Input_1_GPIO_Port GPIOA
#define Led_1_Pin GPIO_PIN_1
#define Led_1_GPIO_Port GPIOA
#define Input_2_Pin GPIO_PIN_2
#define Input_2_GPIO_Port GPIOA
#define Led_2_Pin GPIO_PIN_3
#define Led_2_GPIO_Port GPIOA
#define Led_3_Pin GPIO_PIN_4
#define Led_3_GPIO_Port GPIOA
#define Input_3_Pin GPIO_PIN_5
#define Input_3_GPIO_Port GPIOA
#define Ainput_1_Pin GPIO_PIN_6
#define Ainput_1_GPIO_Port GPIOA
#define Ainput_2_Pin GPIO_PIN_7
#define Ainput_2_GPIO_Port GPIOA
#define Ainput_3_Pin GPIO_PIN_11
#define Ainput_3_GPIO_Port GPIOA
#define Relay_3_Pin GPIO_PIN_12
#define Relay_3_GPIO_Port GPIOA
#define Relay_2_Pin GPIO_PIN_6
#define Relay_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ClearBit(A, k) (A &= ~(1UL << k))
#define SetBit(A,k)    (A |= 1UL << k)
#define TestBit(A,k) ((A >> k) & 1UL)
#define ToggleBit(A,k)  ( A ^= 1UL << k)
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))

// обработка кнопок
typedef struct
{
	uint32_t input_1; /* вход 1 */
	uint32_t input_2; /* вход 2 */
	uint32_t input_3; /* вход 3 */
	uint32_t time_privat[8]; /*время срабатывания мс*/
	uint32_t R_trig_privat;
	uint32_t input_1_time; /* время срабатывания */
	uint32_t input_2_time; /* время срабатывания */
	uint32_t input_3_time; /*время срабатывания*/
	uint8_t input_1_fall; /* отпускание входа */
	uint8_t input_2_fall; /* отпускание входа */
	uint8_t input_3_fall; /* отпускание входа */
	uint8_t input_1_rise; /* нажатие входа */
	uint8_t input_2_rise; /* нажатие входа */
	uint8_t input_3_rise; /* нажатие входа */
	uint8_t any_input; /*включен любой вход*/
} input_DataType;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
