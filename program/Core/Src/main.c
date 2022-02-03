/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	uint16_t adc_buff[4];
	float U_A_Side 		= 0;
	float U_B_Side 		= 0;
	float I_B_Side 		= 0;
	float U_Input		= 0;
	float vref			= 0;
	float vin			= 0;
	float vout			= 0;
	float iout			= 0;
	float error			= 0;
	float errorI		= 0;
	float vL 			= 0;
	float vH 			= 0;
	float vM			= 0;
	float vboost		= 0;
	float vbuck			= 0;
	uint16_t pwmbuck	= 0;
	uint16_t pwmboost	= 0;
	uint8_t mode 	 	= 0;
	uint8_t direction	= 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
long mapBound(long x, long in_min, long in_max, long out_min, long out_max) {
	if (x > in_max) return out_max;
	else if (x < in_min) return out_min;
	else return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	U_Input		= (adc_buff[0] * 0.00080566); // from 0 to 3.2991777 V
	I_B_Side 	= (adc_buff[1] * 0.00080566) / 0.33;
	U_A_Side	= (adc_buff[2] * 0.00080566) * 11;
	U_B_Side	= (adc_buff[3] * 0.00080566) * 15.70588235;
}
void ADC_SetActiveChannel(ADC_HandleTypeDef *hadc, uint32_t AdcChannel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = AdcChannel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
   Error_Handler();
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_DMA_Init();		// Before ADC_Init !!
  MX_ADC_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();


  HAL_Delay(100);
  HAL_ADC_Start_DMA(&hadc, (uint16_t *)adc_buff, 4);

  //lHAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // turn on complementary channel
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); // turn on complementary channel

  float P = 0.1;
  float I = 0.1;
  float D = 0;

  direction = HAL_GPIO_ReadPin(Dir_Select_GPIO_Port, Dir_Select_Pin);
  mode 		= HAL_GPIO_ReadPin(Mode_Select_GPIO_Port, Mode_Select_Pin);

  while (1)
  {
	  // WORKING STEP-DOWN OPEN LOOP
	  //uint16_t vref = (adc_buff[0]/4 - 512) * 2; // 0 to 1023
	  //if (vref < 0 ) vref = 0;
	  //if (vref > (1023 - 256)) vref = 1023 - 256;	// -256 cause bootstrap voltage
	  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, vref );
	  //HAL_Delay(100);

	  // PROGRESS STEP-DOWN CLOSE LOOP
	  //vref = adc_buff[0];


	  vH	= 600;
	  vL	= 000;
	  vM 	= 500;
	  vref 	= map(adc_buff[0],0,4095,0,2500);		// 0(0) - 24V(2500) divided to 0(0) - 3.3V(4095)
	  iout  = map(adc_buff[1],0,4095,0,10000);		// 0(0) - 10A(1000) divided to 0(0) - 3.3V(4095)
	  if (direction == 0){
		  vin	= map(adc_buff[2],0,4095,0,3630);		// 0(0) - 36.3V(3630) divided to 0(0) - 3.3V(4095)
		  vout 	= map(adc_buff[3],0,4095,0,5183);		// 0(0) - 51.83V(5183) divided to 0(0) - 3.3V(4095)
	  }else {
		  vout	= map(adc_buff[2],0,4095,0,3630);		// 0(0) - 36.3V(3630) divided to 0(0) - 3.3V(4095)
		  vin 	= map(adc_buff[3],0,4095,0,5183);		// 0(0) - 51.83V(5183) divided to 0(0) - 3.3V(4095)
	  }
	  // Error calculation
	  error	= vref - vout;
	  errorI += error;

	  vbuck = error * P + errorI * I;
	  vboost = vbuck - vM;

	  pwmbuck 	= mapBound(vbuck,vL,vH,0,1023 - 256);	// SAFETY - T1 bootstrap voltage
	  pwmboost  = mapBound(vboost,vL,vH,0,1023 - 256);	// SAFETY - T3 bootstrap voltage, boost max output voltage

	  if (direction == 0){
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwmbuck );
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwmboost );
	  }else {
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1023 - pwmbuck );
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1023 - pwmboost);
	  }

	  ///////////////
	  if (vin < 1000){
		    errorI = 0;
		  HAL_GPIO_WritePin(Under_Voltage_GPIO_Port, Under_Voltage_Pin, 1);
	  } else HAL_GPIO_WritePin(Under_Voltage_GPIO_Port, Under_Voltage_Pin, 0);
	  if (vin > 2500){
		  HAL_GPIO_WritePin(Over_Voltage_GPIO_Port, Over_Voltage_Pin, 1);
	  } else HAL_GPIO_WritePin(Over_Voltage_GPIO_Port, Over_Voltage_Pin, 0);
	  HAL_Delay(1);
  }
  return 0;
  /*########################################################*/
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_ADC_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
