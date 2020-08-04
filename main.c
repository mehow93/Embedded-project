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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MESSAGE_LENGTH 4
#define FIRST_CHAR_TO_SEND 3
#define LAST_BIT_SHIFTED 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
char send_buffer[MESSAGE_LENGTH]; // to hold input string
uint8_t size; // size of send_buffer[]

enum state_machine
{
	IDLE,
	TRANSMITTING,
	SENDING_TO_PIN,
	CHECKING_CHARS,


}typedef state_of_transmition_type; // state machine to control transmittion

enum sending_state_machine
{
	SETTING_START_BIT,
	DECODING,
	SETTING_END_BIT
}typedef sending_state_machine_type;

state_of_transmition_type state_of_transmition = TRANSMITTING;
sending_state_machine_type state_of_sending  = SETTING_START_BIT;

// this to variables later make as static local variables in HAL_TIM_PeriodElapsedCallback()
uint8_t shift_counter = 0; // check how many shifts were made
int8_t msg_counter = FIRST_CHAR_TO_SEND; // to choose char from message[]
//delete later
uint8_t test_zero;
uint8_t test_one;
uint8_t test_two;
uint8_t test_three;
uint8_t send_counter = 0;
uint8_t stupid_var = 0;

void Send_To_Pin(uint8_t state); // change '0' and '1' to high and low states
void Send_Message(void); // send message
uint8_t Get_Lowest_Bit(uint8_t value); // get lowest bit from value
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)// interrupt from timer
{
	uint8_t bit_shift = 0x01; // shift 1 bit
	if(htim->Instance == TIM10 && state_of_transmition == SENDING_TO_PIN)// if interrupt from timer10 occurs and state machine is ready
 	{
		stupid_var = 1;
		if(state_of_sending == SETTING_START_BIT) // this is start of transmitting
	 	{
	 		HAL_GPIO_WritePin(COMMUNICATION_PIN_GPIO_Port,COMMUNICATION_PIN_Pin, GPIO_PIN_SET);
	 		state_of_sending = DECODING;
	 		stupid_var = 2;

	 	}
	 	else if (state_of_sending == SETTING_END_BIT) // this is end of transmiiting - set Low state
	 	{
	 		HAL_GPIO_WritePin(COMMUNICATION_PIN_GPIO_Port,COMMUNICATION_PIN_Pin, GPIO_PIN_RESET);
			if(msg_counter == -1)
			{

				state_of_transmition = TRANSMITTING; // set state machine to checking chars
				msg_counter = FIRST_CHAR_TO_SEND;
				stupid_var = 3;
			}
	 		state_of_sending = SETTING_START_BIT; // back to first state
	 		stupid_var = 11;
	 	}
	 	else //start decoding
	 	{
	 		uint8_t result = Get_Lowest_Bit(send_buffer[msg_counter]); //send first char to masking
	 		Send_To_Pin(result); // changes state of pin
	 		send_buffer[msg_counter] = send_buffer[msg_counter] >> bit_shift;// shift char by one bit
	 		shift_counter++; // add one shift
	 		HAL_GPIO_TogglePin(GPIOD,GREEN_LIGHT_Pin);
			if(shift_counter == LAST_BIT_SHIFTED) // if one char is whole masked and send to pin
			{
				shift_counter = 0;// reset shift counter
				msg_counter--; // // move to next char in message[]
				state_of_sending = SETTING_END_BIT; // enable end of transmition
				stupid_var = 4;
			}



	 	}

 	 }
}


void Send_Message(void)
{
	stupid_var = 14;
	size = sprintf(send_buffer, "TEST");// to know how many chars are in send_buffer
	stupid_var = 5;
	test_zero=send_buffer[0];
	test_one=send_buffer[1];
	test_two=send_buffer[2];
	test_three=send_buffer[3];
	send_counter++;
	state_of_transmition = SENDING_TO_PIN; // start Send_To_Pin() in timer10 interrupt

}
void Send_To_Pin(uint8_t state)
{
//test
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



//test
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(state_of_transmition == TRANSMITTING)
	  	{
			stupid_var = 10;
			HAL_GPIO_TogglePin(GPIOD,ORANGE_LIGHT_Pin);
			Send_Message();


	  	}
		stupid_var = 13;
	}
//sgf
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
  htim10.Init.Prescaler = 4;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 499;
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
