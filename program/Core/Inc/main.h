/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
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
#include "stm32f0xx_hal.h"

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
#define U_L_COMP_Pin GPIO_PIN_13
#define U_L_COMP_GPIO_Port GPIOC
#define ADC_Pin GPIO_PIN_0
#define ADC_GPIO_Port GPIOA
#define IS_boost_COMP_Pin GPIO_PIN_1
#define IS_boost_COMP_GPIO_Port GPIOA
#define I_OUT_ADC_Pin GPIO_PIN_2
#define I_OUT_ADC_GPIO_Port GPIOA
#define U_IN_ADC_Pin GPIO_PIN_3
#define U_IN_ADC_GPIO_Port GPIOA
#define U_OUT_ADC_Pin GPIO_PIN_4
#define U_OUT_ADC_GPIO_Port GPIOA
#define U_OUT_COMP_Pin GPIO_PIN_5
#define U_OUT_COMP_GPIO_Port GPIOA
#define I_OUT_COMP_Pin GPIO_PIN_6
#define I_OUT_COMP_GPIO_Port GPIOA
#define UB_ctrl_Pin GPIO_PIN_7
#define UB_ctrl_GPIO_Port GPIOA
#define UD_ctrl_Pin GPIO_PIN_0
#define UD_ctrl_GPIO_Port GPIOB
#define UE_ctrl_Pin GPIO_PIN_1
#define UE_ctrl_GPIO_Port GPIOB
#define I_OUT_COMPB2_Pin GPIO_PIN_2
#define I_OUT_COMPB2_GPIO_Port GPIOB
#define Over_Voltage_Pin GPIO_PIN_12
#define Over_Voltage_GPIO_Port GPIOB
#define Under_Voltage_Pin GPIO_PIN_13
#define Under_Voltage_GPIO_Port GPIOB
#define UA_ctrl_Pin GPIO_PIN_8
#define UA_ctrl_GPIO_Port GPIOA
#define UC_ctrl_Pin GPIO_PIN_9
#define UC_ctrl_GPIO_Port GPIOA
#define Dir_Select_Pin GPIO_PIN_6
#define Dir_Select_GPIO_Port GPIOF
#define Mode_Select_Pin GPIO_PIN_7
#define Mode_Select_GPIO_Port GPIOF
#define I_OUT_COMP_PWM_Pin GPIO_PIN_4
#define I_OUT_COMP_PWM_GPIO_Port GPIOB
#define U_OUT_COMP_PWM_Pin GPIO_PIN_5
#define U_OUT_COMP_PWM_GPIO_Port GPIOB
#define IS_boost_COMP_PWM_Pin GPIO_PIN_8
#define IS_boost_COMP_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
