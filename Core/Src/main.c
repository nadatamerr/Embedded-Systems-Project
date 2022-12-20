/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#define perpendicular 0
#define parallel 1
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
 TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

volatile int flag[3] , rise[3] , fall[3];
volatile int distance_left;
volatile int distance_right; 
volatile int flag2;
volatile char buffer[100];
volatile int counter;
int parktype[2] = {parallel,parallel};
int time[2]={0,0};
int lastcar[2]={0,0};
int cartime[2]={0,0};


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	
	int i=0;
	
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) 
	{
			i = 1;


		if(!flag[i]){	 // if you’re waiting for the rising edge
			// Capture the timer current value
			rise[i] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
			// Set the polarity to wait for the next fall[i]ing edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1, TIM_CLOCKPOLARITY_FALLING);
				flag[i] = !flag[i]; 
		}
		else { 	// if you’re waiting for the falling edge
			// Capture the timer current value
			fall[i] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
			// Set the polarity to wait for the next rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1, TIM_CLOCKPOLARITY_RISING);
			flag[i] = !flag[i]; 
			// measure the distance_leftbetween the two edges
			uint32_t x = ((fall[i] - rise[i]))/58; 
			fall[i] = 0, rise[i] = 0;

			distance_right = x;
		}
//		char uartBuf4[25];
//    sprintf(uartBuf4, "distance right: %d\r\n", distance_right);
//    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf4, 25, 200);
	}
	
	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
				
			i=0;
			
			if(!flag[i]){	 // if you’re waiting for the rising edge
					// Capture the timer current value
					rise[i] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
					// Set the polarity to wait for the next fall[i]ing edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_2, TIM_CLOCKPOLARITY_FALLING);
						flag[i] = !flag[i]; 
			}
			else { 	// if you’re waiting for the falling edge
				// Capture the timer current value
				fall[i] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
				// Set the polarity to wait for the next rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_2, TIM_CLOCKPOLARITY_RISING);
				flag[i] = !flag[i]; 
				// measure the distance_leftbetween the two edges
				uint32_t x = ((fall[i] - rise[i]))/58; 
				fall[i] = 0, rise[i] = 0;
				
				distance_left = x;
				

			}
		
//		
//		char uartBuf [20];
//		sprintf(uartBuf, "Distance left: %d\r\n",distance_left); 
//		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf,20,20);
	}



}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void reset()
{
  for (int i = 0; i < counter; i++)
    buffer[i] = 0;

  counter = 0;
  flag[0] = 0;
  flag[1] = 0;
  flag[2] = 0;
  distance_left = 0;
  distance_right = 0;
	parktype[0] = parallel;
	parktype[1] = parallel;
	time[0] = 1; 
	time[1] = 1; 
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
	
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  uint8_t f[4] = {0xC2, 0x20, 0xC9, 0x20};
  uint8_t s[4] = {0xC1, 0x0, 0xC9, 0x0};
  // uint8_t f[4] = {0xC2, 0x20, 0xCA, 0x20};
  // uint8_t s [4] = {0xC2, 0x0, 0xCA, 0x0};
  uint8_t back[4] = {0xC1, 0x20, 0xCA, 0x20};
  uint8_t left[4] = {0xC2, 0x00, 0xCA, 0x20};
  uint8_t right[4] = {0xC2, 0x20, 0xC9, 0x0};
  uint8_t right_reverse[4] = {0xC1, 0x00, 0xCA, 0x20};
  uint8_t left_reverse[4] = {0xC1, 0x20, 0xC9, 0x0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  const int meter_time = 5000;           // actual = 4000;
  const int width = 40;                // cm
  const int length = meter_time * 0.6; // cm  (in case of searching for parallel parking)
  const int speed = 10;                // cm/sec

	int park_chosen = -1;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //
    //		while(!flag2);
    //		if (flag2 == 1)
    //		{

    if (buffer[0] == 'S')
    {
			HAL_UART_Transmit(&huart2, f, 4, 100);

      while (1)
      {
        __HAL_TIM_SetCounter(&htim1, 0);
				//channel 2
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
        while (__HAL_TIM_GET_COUNTER(&htim1) < 10);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);

        HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
        HAL_Delay(100);
				
				//channel 1
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
        while (__HAL_TIM_GET_COUNTER(&htim1) < 10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);

        HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
        HAL_Delay(100);
				
				
       //left
        if (distance_left > width)
        {
          time[0]++;
					lastcar[0] = lastcar[0]>cartime[0] ? lastcar[0] : cartime[0];
					cartime[0]=1;
        }
        else
        {
					cartime[0]++;
          time[0] = 1;
        }
				
				//right
				if (distance_right > width)
        {
          time[1]++;
					lastcar[1] = lastcar[1]>cartime[1] ? lastcar[1] : cartime[1];
					cartime[1]=1;
        }
        else
        {
					cartime[1]++;
          time[1] = 1;
        }
				
				//left				
				if(lastcar[0] >8)
					parktype[0] = parallel;
				else
					parktype[0] = perpendicular;

				//right
				if(lastcar[1] >12)
					parktype[1] = parallel;
				else
					parktype[1] = perpendicular;

        // search for parking space
        //left
        if ((time[0]) >= 12 && parktype[0] == perpendicular) 
        {
          HAL_UART_Transmit(&huart2, s, 4, 100);
					HAL_UART_Transmit(&huart2, back, 4, 100);
          HAL_Delay(450);
          HAL_UART_Transmit(&huart2, left_reverse, 4, 100);
          HAL_Delay(3400);
          HAL_UART_Transmit(&huart2, back, 4, 100);
          HAL_Delay(1600);
          HAL_UART_Transmit(&huart2, s, 4, 100);
					park_chosen=1;
//					char uartBuf3[20];
//        sprintf(uartBuf3, "park: %d\r\n", park_chosen);
//        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf3, 20, 200);
				
          reset();
          break;
				}					
				else if(parktype[0] == parallel && time[0] >= 14) {  //parallel
					HAL_UART_Transmit(&huart2, f, 4, 100);
					HAL_Delay(500);
					HAL_UART_Transmit(&huart2, left_reverse, 4, 100);
					HAL_Delay(1800);
					HAL_UART_Transmit(&huart2, back, 4, 100);
					HAL_Delay(1900);
					HAL_UART_Transmit(&huart2, right_reverse, 4, 100);
					HAL_Delay(2150);
					HAL_UART_Transmit(&huart2, s, 4, 100);
					park_chosen=2;
//					char uartBuf3[20];
//        sprintf(uartBuf3, "park: %d\r\n", park_chosen);
//        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf3, 20, 200);
				
					reset();
					break;
				}
     //right
				else if ((time[1]) >= 12 && parktype[1] == perpendicular) 
        {
          HAL_UART_Transmit(&huart2, s, 4, 100);
					HAL_UART_Transmit(&huart2, back, 4, 100);
          HAL_Delay(450);
          HAL_UART_Transmit(&huart2, right_reverse, 4, 100);
          HAL_Delay(4000);
          HAL_UART_Transmit(&huart2, back, 4, 100);
          HAL_Delay(1700);
          HAL_UART_Transmit(&huart2, s, 4, 100);
					park_chosen=3;
//					char uartBuf3[20];
//        sprintf(uartBuf3, "park: %d\r\n", park_chosen);
//        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf3, 20, 200);
				
          reset();
          break;
				}					
				else if(parktype[1] == parallel && time[1] >= 14) {  //parallel
					HAL_UART_Transmit(&huart2, f, 4, 100);
					HAL_Delay(500);
					HAL_UART_Transmit(&huart2, right_reverse, 4, 100);
					HAL_Delay(1800);
					HAL_UART_Transmit(&huart2, back, 4, 100);
					HAL_Delay(1900);
					HAL_UART_Transmit(&huart2, left_reverse, 4, 100);
					HAL_Delay(2000);
					HAL_UART_Transmit(&huart2, s, 4, 100);
					park_chosen=4;
//					char uartBuf3[20];
//        sprintf(uartBuf3, "park: %d\r\n", park_chosen);
//        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf3, 20, 200);
				
					reset();
					break;
				}

//        char uartBuf[20];
//        sprintf(uartBuf, "Time: %d\r\n", time[0]);
//        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, 20, 200);
//				char uartBuf2[20];
//        sprintf(uartBuf2, "Time2: %d\r\n", time[1]);
//        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf2, 20, 200);
//				char uartBuf2[20];
//        sprintf(uartBuf2, "lastcar[0]: %d\r\n", lastcar[0]);
//        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf2, 20, 200);
//				char uartBuf3[20];
//        sprintf(uartBuf3, "parktype[0]: %d\r\n", parktype[0]);
//        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf3, 20, 200);
//				char uartBuf4[25];
//        sprintf(uartBuf4, "distance left: %d\r\n", distance_left);
//        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf4, 25, 200);
//				char uartBuf5[25];
//        sprintf(uartBuf5, "distance right: %d\r\n", distance_right);
//        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf5, 25, 200);
      }
    }

    //		HAL_UART_Transmit(&huart2, s, 4, 100);
    //		HAL_Delay(5000);
    reset();
    //		}
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
