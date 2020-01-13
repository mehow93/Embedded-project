/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MESSAGE_LENGTH 34
#define MAX_CHAR_VALUE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
uint8_t bit_to_set = 0; // counter to set which value from message[] will be written to pin
uint8_t send_buffer[50];
uint8_t message[34]; // array to hold ready to send message
uint8_t binary_data[8]; // buffer to hold values is binary order
uint8_t* start_of_char = &message[1]; // pointer to write data to message[]
uint8_t size;
uint8_t decimal_code;
uint8_t* start_of_first_char = &message[1]; // save address of start of binary order of first char
uint8_t* start_of_second_char = &message[9]; // save address of start of binary order of second char
uint8_t* start_of_third_char = &message[17]; // save address of start of binary order of third char
uint8_t* start_of_fourth_char = &message[25]; // save address of start of binary order of fourth char

enum state_machine
{
	TRANSMITTING,
	NO_TRANSMITTING,
	SENDING_TO_PIN,
	CHECKING_CHARS,


}typedef state_of_transmition; // state machine to control transmittion

state_of_transmition state = NO_TRANSMITTING;
//delate later
uint8_t test_zero;
uint8_t test_one;
uint8_t test_two=0;
uint8_t test_three=0;
uint8_t test_four;
uint8_t test_five;
uint8_t test_six;
uint8_t test_seven;
uint8_t check_first_char=0; // to check if char was correctly change to binary value
uint8_t check_second_char=0; // to check if char was correctly change to binary value
uint8_t check_third_char=0; // to check if char was correctly change to binary value
uint8_t check_fourth_char=0; // to check if char was correctly change to binary value

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);
static void MX_CRC_Init(void);
void Send_To_Pin(void); // change '0' and '1' to high and low states
void Send_Message(void); // send message
void Decimal_To_Binary(uint8_t value); // change decimal to binary
uint8_t Char_To_Decimal(char arg); // change char into decimal value
void Write_Binary_Data_To_Message(void); // write binary_data to message[]
void Build_Message(void); // build message ready to sent
void Check_Chars(void); // check correctness of binary order of  sending chars
uint8_t Binary_Into_Int(uint8_t* ptr); // change binary_data [] to uint value
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)// interrupt from timer
{
	if(htim->Instance == TIM10)// if interrupt from timer10 occurs
 	{
	 	test_two++;
	 	if(state == SENDING_TO_PIN)
	 	{
	 		Send_To_Pin();
	 	}
 	 }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)// interrupt from button
{
	if(state == NO_TRANSMITTING) // prevoius transmition must be ended
	{
		state = TRANSMITTING; // enable transmitting
	}
}
/* USER CODE BEGIN PFP */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM10_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim10);


  test_zero = message[0];
  test_one = message[33];
  check_first_char=1;
  check_second_char=1;
  check_third_char=1;
  check_fourth_char=1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(state == TRANSMITTING)
	  	{
			Send_Message();
	  	}
		else if (state == CHECKING_CHARS)
		{
			Check_Chars();
		}

	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void Send_Message(void)
{
	uint8_t i = 0;
	size = sprintf(send_buffer, "TEST");// to know how many chars are in send_buffer
	for(i =0; i < size; i++)
	{
		Decimal_To_Binary(send_buffer[i]);
		Write_Binary_Data_To_Message();

	}
	Build_Message();
	state = SENDING_TO_PIN; // start Send_To_Pin() in timer10 interrupt

}
void Send_To_Pin(void)
{

	if(bit_to_set < MESSAGE_LENGTH) // to check if program is not out of message[] boundaries
	{
		if (message[bit_to_set] == 0) // if field of array equals to '0' set state of pin to low
		{
			HAL_GPIO_WritePin(COMMUNICATION_PIN_GPIO_Port,COMMUNICATION_PIN_Pin, GPIO_PIN_RESET);
		}
		else // if field of array equals to  '1' set state of pin to high
		{
			HAL_GPIO_WritePin(COMMUNICATION_PIN_GPIO_Port,COMMUNICATION_PIN_Pin, GPIO_PIN_SET);
		}
		bit_to_set++; // move to next 'bit'
	}
	else
	{
		bit_to_set = 0;
		state = CHECKING_CHARS;
	}
}

void Decimal_To_Binary(uint8_t value)
{
	int i = 7;
	if ((value != 0) && (value < MAX_CHAR_VALUE))
	{
		while (value != 0)
		{
			binary_data[i] = value % 2;
			value = value >> 1;
			i--;
		}
	}
	else
	{
		for (int j = 0; j < 8; j++)
		{
			binary_data[j] = 0;
		}
	}
}
void Write_Binary_Data_To_Message(void)
{
	memcpy(start_of_char,binary_data,8); // coping binary_data to message
	if (start_of_char != NULL) // improve this condition, mind that ptr can point out of message[] boundaries
	{
		start_of_char = start_of_char+8; // move pointer to next place to save binary_data[]

	}
}

void Build_Message(void)
{
	message[0] = 1; // 1 in first cell of array has to force high state to start transmitting
	message[33] = 0; // 0 in last cell of array has to force low state to end transmitting
}
uint8_t Binary_Into_Int(uint8_t* ptr)
{
	int i;
	int result=0;
	int exponent=7;
	for(i=0;i<8;i++)
	{
		if((*ptr) == 1)
		{
			result += pow(2,exponent);
		}
		else
		{
			//do nothing
		}
		ptr++;
		exponent--;

	}
	return result;
}

void Check_Chars(void)
{
	check_first_char = Binary_Into_Int(start_of_first_char);
	check_second_char = Binary_Into_Int(start_of_second_char);
	check_third_char = Binary_Into_Int(start_of_third_char);
	check_fourth_char = Binary_Into_Int(start_of_fourth_char);

	state = NO_TRANSMITTING; // end transmittion
}
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
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  htim10.Init.Period = 9998;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GREEN_LIGHT_Pin|ORANGE_LIGHT_Pin|COMMUNICATION_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_LIGHT_Pin ORANGE_LIGHT_Pin COMMUNICATION_PIN_Pin */
  GPIO_InitStruct.Pin = GREEN_LIGHT_Pin|ORANGE_LIGHT_Pin|COMMUNICATION_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
