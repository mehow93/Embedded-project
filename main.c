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

#define MESSAGE_LENGTH 6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
//test
uint8_t check_tab_counter = 0; // counter to set go throw check[]
uint8_t send_buffer[4]; // to hold input string
uint8_t binary_data[8]; // buffer to hold values is binary order
uint8_t check[32];
uint8_t size;
uint8_t decimal_code;
uint8_t* start_of_first_char = &check[31]; // save address of start of binary order of first char
uint8_t* start_of_second_char = &check[23]; // save address of start of binary order of second char
uint8_t* start_of_third_char = &check[15]; // save address of start of binary order of third char
uint8_t* start_of_fourth_char = &check[7]; // save address of start of binary order of fourth char

enum state_machine
{
	TRANSMITTING,
	NO_TRANSMITTING,
	SENDING_TO_PIN,
	CHECKING_CHARS,


}typedef state_of_transmition; // state machine to control transmittion
enum sending_state_machine
{
	SETTING_START_BIT,
	DECODING,
	SETTING_END_BIT
}typedef sending_state_machine;

state_of_transmition state = NO_TRANSMITTING;
sending_state_machine sending_state  = SETTING_START_BIT;
// this to variables later make as static local variables in HAL_TIM_PeriodElapsedCallback()
uint8_t shift_counter = 0; // check how many shifts were made
int8_t msg_counter = 3; // to choose char from message[]
//delate later
uint8_t test_zero;
uint8_t test_one;
uint8_t test_two;
uint8_t test_three;
uint8_t test_four;
uint8_t test_five;
uint8_t test_six;
uint8_t test_seven;
uint8_t check_first_char=0; // to check if char was correctly change to binary value
uint8_t check_second_char=0; // to check if char was correctly change to binary value
uint8_t check_third_char=0; // to check if char was correctly change to binary value
uint8_t check_fourth_char=0; // to check if char was correctly change to binary value

void Send_To_Pin(uint8_t state); // change '0' and '1' to high and low states
void Send_Message(void); // send message
uint8_t Get_Lowest_Bit(uint8_t value); // get lowest bit from value
uint8_t Char_To_Decimal(char arg); // change char into decimal value
void Build_Message(void); // build message ready to sent
void Check_Chars(void); // check correctness of binary order of  sending chars
uint8_t Binary_Into_Int(uint8_t* ptr); // change binary_data [] to uint value
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)// interrupt from timer
{

	uint8_t bit_shift = 0x01; // shift 1 bit

	if(htim->Instance == TIM10)// if interrupt from timer10 occurs
 	{

	 	if(state == SENDING_TO_PIN)
	 	{
	 		if(sending_state == SETTING_START_BIT) // this is start of transmitting
	 		{
	 			HAL_GPIO_WritePin(COMMUNICATION_PIN_GPIO_Port,COMMUNICATION_PIN_Pin, GPIO_PIN_SET);
	 			sending_state = DECODING;
	 		}
	 		else if (sending_state == SETTING_END_BIT) // this is end of transmiiting - set Low state
	 		{
	 			HAL_GPIO_WritePin(COMMUNICATION_PIN_GPIO_Port,COMMUNICATION_PIN_Pin, GPIO_PIN_RESET);
	 			msg_counter =3; // restart reading from send_buffer[]
	 			state = CHECKING_CHARS; // set state machine to checking chars

	 			sending_state = SETTING_START_BIT; // back to first state
	 		}
	 		else //start decoding
	 		{
	 			uint8_t result = Get_Lowest_Bit(send_buffer[msg_counter]); //send first char to masking
	 			Send_To_Pin(result); // changes state of pin
	 			send_buffer[msg_counter] = send_buffer[msg_counter] >> bit_shift;// shift char by one bit
	 			shift_counter++; // add one shift
	 			check[check_tab_counter] = result;
	 			check_tab_counter++;
				if(shift_counter == 8) // if one char is whole masked and send to pin
				{
					shift_counter =0;// reset shift counter
					msg_counter--; // move to next char in message[]
				}
				else
				{
					//do nothing
				}
				if(msg_counter == -1)
				{
					sending_state = SETTING_END_BIT; // enable end of transmition

				}
				else
				{
					// do nothing
				}

	 		}
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

void Send_Message(void)
{

	size = sprintf(send_buffer, "TEST");// to know how many chars are in send_buffer
	test_zero=send_buffer[0];
	test_one=send_buffer[1];
	test_two=send_buffer[2];
	test_three=send_buffer[3];

	state = SENDING_TO_PIN; // start Send_To_Pin() in timer10 interrupt

}
void Send_To_Pin(uint8_t state)
{

	if (state == 0) // if field of array equals to '0' set state of pin to low
	{
		HAL_GPIO_WritePin(COMMUNICATION_PIN_GPIO_Port,COMMUNICATION_PIN_Pin, GPIO_PIN_RESET);
	}
	else // if field of array equals to  '1' set state of pin to high
	{
		HAL_GPIO_WritePin(COMMUNICATION_PIN_GPIO_Port,COMMUNICATION_PIN_Pin, GPIO_PIN_SET);
	}

}

uint8_t Get_Lowest_Bit(uint8_t value)
{
	return(value & 0x01);
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
		ptr--;
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
	check_tab_counter=0;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);
static void MX_CRC_Init(void);

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
  htim10.Init.Period = 9;
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
