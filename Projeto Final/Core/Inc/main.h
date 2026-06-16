/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Motor_Esq_PWM_Pin GPIO_PIN_0
#define Motor_Esq_PWM_GPIO_Port GPIOC
#define Motor_Dir_PWM_Pin GPIO_PIN_1
#define Motor_Dir_PWM_GPIO_Port GPIOC
#define IR1_AD_Pin GPIO_PIN_0
#define IR1_AD_GPIO_Port GPIOA
#define IR_Fr_AD_Pin GPIO_PIN_1
#define IR_Fr_AD_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define LED_Y_Pin GPIO_PIN_5
#define LED_Y_GPIO_Port GPIOA
#define IR2_AD_Pin GPIO_PIN_6
#define IR2_AD_GPIO_Port GPIOA
#define Tensao_Bateria_Pin GPIO_PIN_7
#define Tensao_Bateria_GPIO_Port GPIOA
#define LCD_I2C_SCL_Pin GPIO_PIN_4
#define LCD_I2C_SCL_GPIO_Port GPIOC
#define BT_Enter_Pin GPIO_PIN_5
#define BT_Enter_GPIO_Port GPIOC
#define Ultra_Trig_PWM_Pin GPIO_PIN_2
#define Ultra_Trig_PWM_GPIO_Port GPIOB
#define HC05_RX_Pin GPIO_PIN_10
#define HC05_RX_GPIO_Port GPIOB
#define HC05_TX_Pin GPIO_PIN_11
#define HC05_TX_GPIO_Port GPIOB
#define Motor_Dir_IN3_Pin GPIO_PIN_12
#define Motor_Dir_IN3_GPIO_Port GPIOB
#define IR3_AD_Pin GPIO_PIN_13
#define IR3_AD_GPIO_Port GPIOB
#define IR4_AD_Pin GPIO_PIN_15
#define IR4_AD_GPIO_Port GPIOB
#define Ultra_Eco_TIM_Pin GPIO_PIN_6
#define Ultra_Eco_TIM_GPIO_Port GPIOC
#define BT_Baixo_Pin GPIO_PIN_7
#define BT_Baixo_GPIO_Port GPIOC
#define BT_Esq_Pin GPIO_PIN_8
#define BT_Esq_GPIO_Port GPIOC
#define BT_Dir_Pin GPIO_PIN_9
#define BT_Dir_GPIO_Port GPIOC
#define LCD_I2C_SDA_Pin GPIO_PIN_8
#define LCD_I2C_SDA_GPIO_Port GPIOA
#define IR5_AD_Pin GPIO_PIN_9
#define IR5_AD_GPIO_Port GPIOA
#define Motor_Esq_IN2_Pin GPIO_PIN_10
#define Motor_Esq_IN2_GPIO_Port GPIOA
#define LED_R_PWM_Pin GPIO_PIN_11
#define LED_R_PWM_GPIO_Port GPIOA
#define LED_G_PWM_Pin GPIO_PIN_12
#define LED_G_PWM_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define Buzzer_PWM_Pin GPIO_PIN_15
#define Buzzer_PWM_GPIO_Port GPIOA
#define Switch_Fr_Pin GPIO_PIN_2
#define Switch_Fr_GPIO_Port GPIOD
#define Switch_Fr_EXTI_IRQn EXTI2_IRQn
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define Encoder_Esq_TIM_Pin GPIO_PIN_4
#define Encoder_Esq_TIM_GPIO_Port GPIOB
#define Encoder_Dir_TIM_Pin GPIO_PIN_5
#define Encoder_Dir_TIM_GPIO_Port GPIOB
#define BT_Cima_Pin GPIO_PIN_6
#define BT_Cima_GPIO_Port GPIOB
#define Moto_Esq_IN1_Pin GPIO_PIN_7
#define Moto_Esq_IN1_GPIO_Port GPIOB
#define LED_B_PWM_Pin GPIO_PIN_8
#define LED_B_PWM_GPIO_Port GPIOB
#define Motor_Dir_IN4_Pin GPIO_PIN_9
#define Motor_Dir_IN4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
