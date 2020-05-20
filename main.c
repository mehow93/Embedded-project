/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim10;
enum recieve_state_machine
{ // states of receiving data
		IDLE,
		RECEIVING,
		CHECKING

}typedef recieve_state_type;
recieve_state_type recieve_state = IDLE;
uint8_t msg_cnt = 3;// counter to go throw msg[]
uint8_t bit_cnt = 0;// counter to
uint8_t msg[39]; // to store RT_PIN states
uint8_t ready_msg[4]; // to store ready chars
uint8_t bit_shift = 0; // to move one state of pin
uint8_t result = 0x00; // to store char

uint8_t first_char = 0; // variables to check correctness of decoding. To delete later
uint8_t second_char = 0;
uint8_t third_char = 0;
uint8_t fourth_char = 0;



/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
void Read_Pin_Status(); // read states of RT_PIN
uint8_t Binary_Into_Int(uint8_t* ptr); // change binary data to int type
void Decode();// change data in msg[] to decimal value
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if((htim->Instance == TIM10) && (recieve_state ==  RECEIVING)) // Interrupt from TIMER10 and enum machine is in receiving state
	{
		Read_Pin_Status(); // start reading states
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // interrupt from high RT_PIN
{
	if((GPIO_Pin == RT_PIN_Pin) && (recieve_state == IDLE))
	{
		recieve_state = RECEIVING;// start recieving data
	}
}

void Read_Pin_Status() // read status of RT_PIN
{

	if(bit_cnt == 9)
	{

		recieve_state = CHECKING;
		bit_cnt = 0;
	}
	else
	{

		uint8_t one_state = HAL_GPIO_ReadPin(RT_PIN_GPIO_Port, RT_PIN_Pin) == GPIO_PIN_SET ? 1:0; // write 1 if pin state is high, and 0 if it is low.
		one_state = one_state << bit_shift; // set bit in right place
		result = result | one_state; // add bit to whole char
		bit_shift++;
		if(bit_shift == 7) // if whole char is received
		{
			ready_msg[msg_cnt] = result;
			bit_shift = 0; // set bit_shift to lowest bit
			msg_cnt --; // go to next cell in ready_msg[]
			result = 0x00; // reset result to delete later
		}
		bit_cnt++ ; //count every bit
	}

}
uint8_t Binary_Into_Int(uint8_t* single_state)
{
	uint8_t result = 0;
	uint8_t bit = 0; // to store single bit
	uint8_t bit_mask = 0x01; // bit mask to get exactly one bit

    for(uint8_t current_bit = 0; current_bit < 8; current_bit ++) //loop iterates 8 times (one iterate per one bit)
    {
      //  bit = x & bit_mask; // get single bit
        result = result | bit; // write bit to result
        bit_mask = bit_mask << 1; // move to 9next bit
    }

}
void Decode()// change data in msg[] to decimal value
{

	uint8_t* binary_data_start = &msg[8]; // start binary data of first char
	uint8_t ready_msg_counter =3; // counter to go throw ready_msg[]
	ready_msg[ready_msg_counter] = Binary_Into_Int(binary_data_start); // decode first char
	ready_msg_counter --; // move to next cell for new char
	binary_data_start += 10; // move to next start of binary data

	ready_msg[ready_msg_counter] = Binary_Into_Int(binary_data_start); // decode second char
	ready_msg_counter --; // move to next cell for new char
	binary_data_start += 10; // move to next start of binary data

	ready_msg[ready_msg_counter] = Binary_Into_Int(binary_data_start); // decode third char
	ready_msg_counter --; // move to next cell for new char
	binary_data_start +=10; // move to next start of binary data

	ready_msg[ready_msg_counter] = Binary_Into_Int(binary_data_start); // decode fourth char
	binary_data_start = &msg[8]; // back to first char

}

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM10_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		 if(recieve_state == CHECKING)
		  {
			  first_char = ready_msg[0];
			  second_char = ready_msg[1];
			  third_char = ready_msg[2];
			  fourth_char = ready_msg[3];
			  recieve_state = IDLE;
			  msg_cnt = 3; // set to last cell

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 9999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 154;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|LED_GREEN_Pin|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD11 LED_GREEN_Pin PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|LED_GREEN_Pin|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RT_PIN_Pin */
  GPIO_InitStruct.Pin = RT_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RT_PIN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
