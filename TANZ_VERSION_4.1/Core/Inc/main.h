/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l0xx_hal.h"

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
#define in1_zoom_Pin GPIO_PIN_3
#define in1_zoom_GPIO_Port GPIOA
#define in2_zoom_Pin GPIO_PIN_4
#define in2_zoom_GPIO_Port GPIOA
#define dir_tilt_Pin GPIO_PIN_6
#define dir_tilt_GPIO_Port GPIOA
#define en_zoom_Pin GPIO_PIN_7
#define en_zoom_GPIO_Port GPIOA
#define dir_pan_Pin GPIO_PIN_0
#define dir_pan_GPIO_Port GPIOB
#define t1_sw_Pin GPIO_PIN_1
#define t1_sw_GPIO_Port GPIOB
#define t2_sw_Pin GPIO_PIN_8
#define t2_sw_GPIO_Port GPIOA
#define p_sw_Pin GPIO_PIN_11
#define p_sw_GPIO_Port GPIOA
#define in1_Pin GPIO_PIN_12
#define in1_GPIO_Port GPIOA
#define debug_Pin GPIO_PIN_3
#define debug_GPIO_Port GPIOB
#define enab_tilt_Pin GPIO_PIN_4
#define enab_tilt_GPIO_Port GPIOB
#define enab_pan_Pin GPIO_PIN_5
#define enab_pan_GPIO_Port GPIOB
//#define in4_Pin GPIO_PIN_6
//#define in4_GPIO_Port GPIOB
//#define in3_Pin GPIO_PIN_7
//#define in3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
