/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void display7SEG_groupA(int num){
	HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, RESET);
	HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, RESET);
	HAL_GPIO_WritePin(c_GPIO_Port, c_Pin, RESET);
	HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, RESET);
	HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, RESET);
	HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, RESET);
	HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, RESET);

	if(num == 0 ){
		HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, SET);
	}
	if(num == 1 ){
		HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, SET);
		HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, SET);
		HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, SET);
		HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, SET);
		HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, SET);
	}
	if(num == 2 ){
		HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, SET);
		HAL_GPIO_WritePin(c_GPIO_Port, c_Pin, SET);
	}
	if(num == 3 ){
		HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, SET);
		HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, SET);
	}
	if(num == 4 ){
		HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, SET);
		HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, SET);
		HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, SET);
	}
	if(num == 5 ){
			HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, SET);
			HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, SET);
	}
	if(num == 6 ){
		HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, SET);
	}
	if(num == 7 ){
		HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, SET);
		HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, SET);
		HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, SET);
		HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, SET);
	}
	if(num==8){
	}
	if(num == 9 ){
		HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, SET);
	}

}
void display7SEG_groupB(int num){
	HAL_GPIO_WritePin(PB7_GPIO_Port, PB7_Pin, RESET);
	HAL_GPIO_WritePin(PB8_GPIO_Port, PB8_Pin, RESET);
	HAL_GPIO_WritePin(PB9_GPIO_Port, PB9_Pin, RESET);
	HAL_GPIO_WritePin(PB10_GPIO_Port, PB10_Pin, RESET);
	HAL_GPIO_WritePin(PB11_GPIO_Port, PB11_Pin, RESET);
	HAL_GPIO_WritePin(PB12_GPIO_Port, PB12_Pin, RESET);
	HAL_GPIO_WritePin(PB13_GPIO_Port, PB13_Pin, RESET);
	if(num == 0){
		HAL_GPIO_WritePin(PB13_GPIO_Port, PB13_Pin, SET);
	}
	if (num == 1){
		HAL_GPIO_WritePin(PB13_GPIO_Port, PB13_Pin, SET);
		HAL_GPIO_WritePin(PB7_GPIO_Port, PB7_Pin, SET);
		HAL_GPIO_WritePin(PB12_GPIO_Port, PB12_Pin, SET);
		HAL_GPIO_WritePin(PB10_GPIO_Port, PB10_Pin, SET);
		HAL_GPIO_WritePin(PB11_GPIO_Port, PB11_Pin, SET);
	}
	if (num == 2){
		HAL_GPIO_WritePin(PB12_GPIO_Port, PB12_Pin, SET);
		HAL_GPIO_WritePin(PB9_GPIO_Port, PB9_Pin, SET);
	}
	if (num == 3){
		HAL_GPIO_WritePin(PB12_GPIO_Port, PB12_Pin, SET);
		HAL_GPIO_WritePin(PB11_GPIO_Port, PB11_Pin, SET);
	}
	if (num == 4){
		HAL_GPIO_WritePin(PB7_GPIO_Port, PB7_Pin, SET);
		HAL_GPIO_WritePin(PB11_GPIO_Port, PB11_Pin, SET);
		HAL_GPIO_WritePin(PB10_GPIO_Port, PB10_Pin, SET);
	}
	if (num == 5){
		HAL_GPIO_WritePin(PB8_GPIO_Port, PB8_Pin, SET);
		HAL_GPIO_WritePin(PB11_GPIO_Port, PB11_Pin, SET);
	}
	if (num == 6){
		HAL_GPIO_WritePin(PB8_GPIO_Port, PB8_Pin, SET);
	}
	if (num == 7){
		HAL_GPIO_WritePin(PB13_GPIO_Port, PB13_Pin, SET);
		HAL_GPIO_WritePin(PB12_GPIO_Port, PB12_Pin, SET);
		HAL_GPIO_WritePin(PB11_GPIO_Port, PB11_Pin, SET);
		HAL_GPIO_WritePin(PB10_GPIO_Port, PB10_Pin, SET);
	}
	if (num == 8){

	}
	if (num == 9){
		HAL_GPIO_WritePin(PB11_GPIO_Port, PB11_Pin, SET);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int Yellow_Count_Group1 = 2;
    int Red_Count_Group1 = 4;
    int Green_Count_Group1 = 3;

    int Yellow_Status_Group1 = 0;
    int Red_Status_Group1 = 0;
    int Green_Status_Group1 = 1;


    int Yellow_Count_Group2 = 1;
    int Red_Count_Group2 = 5;
    int Green_Count_Group2 = 3;

    int Yellow_Status_Group2 = 0;
    int Red_Status_Group2 = 1;
    int Green_Status_Group2 = 0;

    while (1)
    {
     /* USER CODE END WHILE */

     /* USER CODE BEGIN 3 */

  	  //-------begin group1 --------
  	  // in group1, green-led bright first

  	  //green-led will turn on in 3 seconds until green-status=0
  	  if(Green_Status_Group1 == 1)
  	  {
  		  display7SEG_groupA(Green_Count_Group1);
  		  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_SET);
  		  HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_RESET);

  		  if(Green_Count_Group1 == 0)
  		  {
  			 Green_Status_Group1 = 0;// green-led turn off
  			 Green_Count_Group1 = 4;// update time of green-led
  			 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET);
  			 Yellow_Status_Group1 = 1;//yellow-led turn on
  		  }
  		  Green_Count_Group1 --;

  	  }

  	  // yellow-led will turn on in 2 seconds until yellow-status=0
  	  if(Yellow_Status_Group1 == 1)
  	  {
  		  display7SEG_groupA(Yellow_Count_Group1);
  		  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port,LED_YELLOW_Pin,GPIO_PIN_SET);
  		  if(Yellow_Count_Group1 == 0)
  		  {
  			  Yellow_Status_Group1 = 0;// yellow-led turn off
  			  Yellow_Count_Group1 = 3;// update time of yellow-led
  			  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port,LED_YELLOW_Pin,GPIO_PIN_RESET);
  			  Red_Status_Group1 = 1;// red-led turn on
  		  }
  		  Yellow_Count_Group1 --;
  	  }

  	  // red-led will turn on in 5 seconds until red-status=0
  	  if(Red_Status_Group1 == 1)
  	  {
  		  display7SEG_groupA(Red_Count_Group1 + 1);
  		  HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_SET);

  		  if(Red_Count_Group1 == 0)
  		  {
  			  Red_Status_Group1 = 0;// red-led turn off
  			  Red_Count_Group1 = 5;// update time of red-led
  			  Green_Status_Group1 = 1;// green-led turn on
  		  }
  		  Red_Count_Group1 --;
  	  }
  	  //---------end group1 ----------


  	  //-------begin group2 -----------
  	  // in group2, red-led bright first

  	  // red-led will turn on in 5 seconds until red-status=0
  	  if(Red_Status_Group2 == 1)
  	  {
  		  display7SEG_groupB(Red_Count_Group2);
  		  HAL_GPIO_WritePin(RED2_GPIO_Port, RED2_Pin, GPIO_PIN_SET);
  		  HAL_GPIO_WritePin(YELLOW2_GPIO_Port,YELLOW2_Pin,GPIO_PIN_RESET);

  		  if(Red_Count_Group2 == 0)
  		  {
  			 Red_Status_Group2 = 0;// red-led turn off
  			 Red_Count_Group2 = 6;// update time of red-led
  			 HAL_GPIO_WritePin(RED2_GPIO_Port,RED2_Pin,GPIO_PIN_RESET);
  			 Green_Status_Group2 = 1;// green-led turn on
  		  }
  		  Red_Count_Group2 --;
  	  }

  	  //green-led will turn on in 3 seconds until green-status=0
  	  if(Green_Status_Group2 == 1)
  	  {
  		  display7SEG_groupB(Green_Count_Group2);
  		  HAL_GPIO_WritePin(GREEN2_GPIO_Port,GREEN2_Pin,GPIO_PIN_SET);

  		  if(Green_Count_Group2 == 0)
  		  {
  			 Green_Status_Group2 = 0;// green-led turn off
  			 Green_Count_Group2 = 4;// update time of green-led
  			 HAL_GPIO_WritePin(GREEN2_GPIO_Port,GREEN2_Pin,GPIO_PIN_RESET);
  			 Yellow_Status_Group2 = 1;//yellow-led turn on
  		  }
  		  Green_Count_Group2 --;
  	  }

  	  //yellow-led will turn on in 3 seconds until yellow-status=0
  	  if(Yellow_Status_Group2 == 1)
  	  {
  		  display7SEG_groupB(Yellow_Count_Group2 + 1);
  		  HAL_GPIO_WritePin(YELLOW2_GPIO_Port,YELLOW2_Pin,GPIO_PIN_SET);

  		  if(Yellow_Count_Group2 == 0)
  		  {
  			  Yellow_Status_Group2 = 0;// yellow-led turn off
  			  Yellow_Count_Group2 = 2;// update time of yellow-led
  			  Red_Status_Group2 = 1;// red-led turn on
  		  }
  		  Yellow_Count_Group2 --;
  	  }
  	  //---------end group2-------------

  	  HAL_Delay(1000);
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|RED2_Pin
                          |YELLOW2_Pin|GREEN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, a_Pin|b_Pin|c_Pin|PB10_Pin
                          |PB11_Pin|PB12_Pin|PB13_Pin|d_Pin
                          |e_Pin|f_Pin|g_Pin|PB7_Pin
                          |PB8_Pin|PB9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED_Pin LED_YELLOW_Pin LED_GREEN_Pin RED2_Pin
                           YELLOW2_Pin GREEN2_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|RED2_Pin
                          |YELLOW2_Pin|GREEN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : a_Pin b_Pin c_Pin PB10_Pin
                           PB11_Pin PB12_Pin PB13_Pin d_Pin
                           e_Pin f_Pin g_Pin PB7_Pin
                           PB8_Pin PB9_Pin */
  GPIO_InitStruct.Pin = a_Pin|b_Pin|c_Pin|PB10_Pin
                          |PB11_Pin|PB12_Pin|PB13_Pin|d_Pin
                          |e_Pin|f_Pin|g_Pin|PB7_Pin
                          |PB8_Pin|PB9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
