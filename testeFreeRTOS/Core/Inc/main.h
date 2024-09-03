/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
void CallBack_PC10(void);
void CallBack_ExtePC13(void);
void CallBack_PB7(void);
void stopRobot(void);
void fowardRobot(void);
void backwardsRobot(void);
void IR_esquerdo_func(void);
void IR_direito_func(void);
void IR_frente_esquerda_func(void);
void IR_frente_direita_func(void);
void printSensorState(void);
void ajustePWM (void);
int increasePWM(int currentPWM, int gain);
int decreasePWM(int currentPWM, int gain);
#define LED2_Pin LL_GPIO_PIN_5;
#define LED2_GPIO_Port GPIOA;


/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define PRESCALER_VALUE     (uint32_t)(((SystemCoreClock) / 20000000) - 1)
#define  PERIOD_VALUE       (uint32_t)(5000 - 1)
#define  PULSE1_VALUE       (uint32_t)(5000/2 - 1)
#define  PULSE2_VALUE       (uint32_t)(5000/2 -1)

#define PRESCALER_VALUE2     (uint32_t)(((SystemCoreClock) / 5000000) - 1)
#define  PERIOD_VALUE2       (uint32_t)(1000 - 1)
#define  PULSE1_VALUE2       (uint32_t)(1000 - 1)
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Meta_Pin GPIO_PIN_1
#define Meta_GPIO_Port GPIOC
#define Frente_Direito_IR_Pin GPIO_PIN_2
#define Frente_Direito_IR_GPIO_Port GPIOC
#define Frente_Direito_IR_EXTI_IRQn EXTI2_IRQn
#define Frente_Esquerda_IR_Pin GPIO_PIN_3
#define Frente_Esquerda_IR_GPIO_Port GPIOC
#define Frente_Esquerda_IR_EXTI_IRQn EXTI3_IRQn
#define Esquerdo_IR_Pin GPIO_PIN_0
#define Esquerdo_IR_GPIO_Port GPIOA
#define Esquerdo_IR_EXTI_IRQn EXTI0_IRQn
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LEBPC7_Pin GPIO_PIN_7
#define LEBPC7_GPIO_Port GPIOC
#define LED_PA9_Pin GPIO_PIN_9
#define LED_PA9_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define Direito_IR_Pin GPIO_PIN_12
#define Direito_IR_GPIO_Port GPIOC
#define Direito_IR_EXTI_IRQn EXTI15_10_IRQn
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define CallBack_PC10_Pin GPIO_PIN_6
#define CallBack_PC10_GPIO_Port GPIOB
#define CallBack_PC10_EXTI_IRQn EXTI9_5_IRQn
#define CallBack_PB7_Pin GPIO_PIN_7
#define CallBack_PB7_GPIO_Port GPIOB
#define CallBack_PB7_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
