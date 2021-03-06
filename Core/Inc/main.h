/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define CAN_ID_1_Pin GPIO_PIN_1
#define CAN_ID_1_GPIO_Port GPIOA
#define CAN_ID_2_Pin GPIO_PIN_2
#define CAN_ID_2_GPIO_Port GPIOA
#define CAN_TRAFFIC_LED_Pin GPIO_PIN_3
#define CAN_TRAFFIC_LED_GPIO_Port GPIOA
#define EEPROM_LED_Pin GPIO_PIN_4
#define EEPROM_LED_GPIO_Port GPIOA
#define Encoder_LED_Pin GPIO_PIN_5
#define Encoder_LED_GPIO_Port GPIOA
#define POWER_SENSE_Pin GPIO_PIN_6
#define POWER_SENSE_GPIO_Port GPIOA
#define CAN_ID_4_Pin GPIO_PIN_7
#define CAN_ID_4_GPIO_Port GPIOA
#define CAN_ID_8_Pin GPIO_PIN_0
#define CAN_ID_8_GPIO_Port GPIOB
#define CAN_ID_16_Pin GPIO_PIN_1
#define CAN_ID_16_GPIO_Port GPIOB
#define CAN_ID_32_Pin GPIO_PIN_2
#define CAN_ID_32_GPIO_Port GPIOB
#define CAN_ID_64_Pin GPIO_PIN_10
#define CAN_ID_64_GPIO_Port GPIOB
#define CAN_ID_128_Pin GPIO_PIN_11
#define CAN_ID_128_GPIO_Port GPIOB
#define MEM_SDA_Pin GPIO_PIN_12
#define MEM_SDA_GPIO_Port GPIOB
#define MEM_SCL_Pin GPIO_PIN_13
#define MEM_SCL_GPIO_Port GPIOB
#define ENC_X_Pin GPIO_PIN_5
#define ENC_X_GPIO_Port GPIOB
#define ENC_X_EXTI_IRQn EXTI9_5_IRQn
#define ENC_A_Pin GPIO_PIN_6
#define ENC_A_GPIO_Port GPIOB
#define ENC_A_EXTI_IRQn EXTI9_5_IRQn
#define ENC_B_Pin GPIO_PIN_7
#define ENC_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
