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
#define ONECHAR_LENGHT 8
#define MAX_CHAR_VALUE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
uint8_t A[8] = { 0, 1, 1, 0, 0, 0, 0, 1 };
uint8_t send_buffer[50];
uint8_t message[50]; // array to hold ready to send message
uint8_t binary_data[8]; // buffer to hold values is binary order
uint8_t size;
uint8_t decimal_code;
//delate later
uint8_t test_zero;
uint8_t test_one;
uint8_t test_two;
uint8_t test_three;
uint8_t test_four;
uint8_t test_five;
uint8_t test_six;
uint8_t test_seven;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);
void Send_To_Pin(void); // change '0' and '1' to high and low states
void Send_Message(void); // send message
void Decimal_To_Binary(uint8_t value); // change decimal to binary
uint8_t Char_To_Decimal(char arg); // change char into decimal value
void Build_Message(void); // buid message
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	/* USER CODE BEGIN 2 */
	Send_Message();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		//Send_To_Pin();
		//decimal_code = Char_To_Decimal('z');
		Decimal_To_Binary(169);
		test_zero = binary_data[0];
		test_one = binary_data[1];
		test_two = binary_data[2];
		test_three = binary_data[3];
		test_four = binary_data[4];
		test_five = binary_data[5];
		test_six = binary_data[6];
		test_seven = binary_data[7];
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
	size = sprintf(send_buffer, "UUU");// to know how many chars are in send_buffer
	for(int i =0; i < size; i++)
	{
		Decimal_To_Binary(send_buffer[i]);
		Send_To_Pin();
	}

}
void Send_To_Pin(void)
{
	uint8_t i = 0; // counter to get throw table

	for (i = 0; i < ONECHAR_LENGHT; i++) // get throw table of char ang change state of pin
	{
		if (binary_data[i] == 0) // if field of array equals to '0' set state of pin to low
		{
			HAL_GPIO_WritePin(COMMUNICATION_PIN_GPIO_Port,COMMUNICATION_PIN_Pin, GPIO_PIN_RESET);
		}
	    else // if field of array equals to  '1' set state of pin to high
		{
			HAL_GPIO_WritePin(COMMUNICATION_PIN_GPIO_Port,COMMUNICATION_PIN_Pin, GPIO_PIN_SET);
		}
		HAL_Delay(500);
	}
}

void Decimal_To_Binary(uint8_t value) {
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
void Build_Message(void)
{
	message[0] = 1; // force HIGH state to start transmitting
	uint8_t* ptr = &message[1]; // copy binary_data here
	memcpy(ptr,binary_data,8);
}
uint8_t Char_To_Decimal(char arg)
{
	uint8_t result = (uint8_t)arg;
	return result;
}
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

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
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
			GREEN_LIGHT_Pin | ORANGE_LIGHT_Pin | COMMUNICATION_PIN_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : Button_Pin */
	GPIO_InitStruct.Pin = Button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : GREEN_LIGHT_Pin ORANGE_LIGHT_Pin COMMUNICATION_PIN_Pin */
	GPIO_InitStruct.Pin = GREEN_LIGHT_Pin | ORANGE_LIGHT_Pin
			| COMMUNICATION_PIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
