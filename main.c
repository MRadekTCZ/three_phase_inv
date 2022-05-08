/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ST0 0b0000000000000000
#define ST1 0b0000000000110001
#define ST2 0b0000000000100011
#define ST3 0b0000000000000111
#define ST4 0b0000000000001110
#define ST5 0b0000000000011100
#define ST6 0b0000000000111000
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void Write_outputs (int state); //Writing all inputs (from 3 phases) in one moment
void Write_outputs_adv (int state, int phase); //Writing inputs of chosen phase
short int lookup[20] = {0,4,10,16,20,20,16,10,4,0,0,4,10,16,20,20,16,10,4,0}; //Lookup Table-only with 20 values, uc can handle only this speed of switching
int value_adc_read; //adc read
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int debug;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//Uart communication
uint8_t mark, announcement[20];
uint16_t d1_kom;
int wait_for_value[3];
uint8_t value_char[2];
int value_int[2][2]={{0,1},{0,0}};
int diode_mode=0;

int freq_decimal_value=0;



//PWM counting
int i_1,i_2,i_3,j_1,j_2,j_3;
long int delay_phase; //120, 240 degrees delay in phases

int freq_wanted=0;
int freq_actual=0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
short int state=0;  //Transistors State of switching
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
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &mark, 1);

  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_TIM_Base_Start_IT(&htim1);
  //HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //connectivity with uart test
	  if (diode_mode) HAL_GPIO_WritePin(GPIOA, 0x0020, GPIO_PIN_SET);
	  else if (!diode_mode) HAL_GPIO_WritePin(GPIOA, 0x0020, GPIO_PIN_RESET);


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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//UART transmit in case feedback answer is needed
	//HAL_UART_Transmit_IT(&huart6, announcement, d1_kom);
	//Long code, but it can't be shorten with outside made function - rxcpltCallback didnt work with that.
	if(huart->Instance == USART2)
	{

		if(mark == 'o')
			{
				diode_mode=1;

			}

		else if(mark == 's')
			{
				diode_mode=0;
				//freq_actual = 1;
			}
		if(diode_mode)
		{
		if(mark == 'f')
			{

			freq_decimal_value = value_int[0][0]*10+value_int[0][1];
			freq_wanted = freq_decimal_value;
			freq_actual = 50 / freq_wanted;
			// setting new values for timers
			htim10.Instance = TIM10;
		  htim10.Init.Prescaler = 59999;
		  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
		  htim10.Init.Period = 5*freq_actual - 1;
		  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
		  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
		  {
			Error_Handler();
		  }
		  htim11.Instance = TIM11;
		  htim11.Init.Prescaler = 49;
		  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
		  htim11.Init.Period = 5*6*freq_actual - 1;
		  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
		  {
		    Error_Handler();
		  }
		  HAL_TIM_Base_Start_IT(&htim10);
		  HAL_TIM_Base_Start_IT(&htim11);
			}
		//Receiving and saving data module
		else if(mark == 'u')
			{
				wait_for_value[2]=1;
			}

		else if(mark == 'q')
			{
				wait_for_value[0]=2;
			}

		else if(mark == 'a')
			{
				wait_for_value[1]=2;
			}

		else if (mark == '0')
			{
				if (wait_for_value[0]==2)
					{
					value_int[0][0]=0;
					wait_for_value[0]=1;
					}
				else if (wait_for_value[0]==1)
					{
					value_int[0][1]=0;
					wait_for_value[0]=0;
					}
				else if (wait_for_value[0]==0)
					{
					}

				if (wait_for_value[1]==2)
					{
				value_int[1][0]=0;
				wait_for_value[1]=1;
					}
				else if (wait_for_value[1]==1)
					{
					value_int[1][1]=0;
					wait_for_value[1]=0;
					}
				else if (wait_for_value[1]==0)
					{
					}

				if (wait_for_value[2]==1)
					{

					wait_for_value[2]=0;
					}
			}

		else if (mark == '1')
				{
				if (wait_for_value[0]==2)
					{
					value_int[0][0]=1;
					wait_for_value[0]=1;
					}
				else if (wait_for_value[0]==1)
					{
					value_int[0][1]=1;
					wait_for_value[0]=0;
					}
				else if (wait_for_value[0]==0)
					{
					}

				if (wait_for_value[1]==2)
					{
				value_int[1][0]=1;
				wait_for_value[1]=1;
					}
				else if (wait_for_value[1]==1)
					{
					value_int[1][1]=1;
					wait_for_value[1]=0;
					}
				else if (wait_for_value[1]==0)
					{
					}

				if (wait_for_value[2]==1)
					{

					wait_for_value[2]=0;
					}
				}

		else if (mark == '2')
			{
				if (wait_for_value[0]==2)
					{
					value_int[0][0]=2;
					wait_for_value[0]=1;
					}
				else if (wait_for_value[0]==1)
					{
					value_int[0][1]=2;
					wait_for_value[0]=0;
					}
				else if (wait_for_value[0]==0)
					{
					}

				if (wait_for_value[1]==2)
					{
					value_int[1][0]=2;
					wait_for_value[1]=1;
					}
				else if (wait_for_value[1]==1)
					{
					value_int[1][1]=2;
					wait_for_value[1]=0;
					}
				else if (wait_for_value[1]==0)
					{
					}

			}
		else if (mark == '3')
			{
				if (wait_for_value[0]==2)
					{
					value_int[0][0]=3;
					wait_for_value[0]=1;
					}
				else if (wait_for_value[0]==1)
					{
					value_int[0][1]=3;
					wait_for_value[0]=0;
					}
				else if (wait_for_value[0]==0)
					{
					}

				if (wait_for_value[1]==2)
					{
					value_int[1][0]=3;
					wait_for_value[1]=1;
					}
				else if (wait_for_value[1]==1)
					{
					value_int[1][1]=3;
					wait_for_value[1]=0;
					}
				else if (wait_for_value[1]==0)
					{
					}

			}
		else if (mark == '4')
			{
				if (wait_for_value[0]==2)
					{
					value_int[0][0]=4;
					wait_for_value[0]=1;
					}
				else if (wait_for_value[0]==1)
					{
					value_int[0][1]=4;
					wait_for_value[0]=0;
					}
				else if (wait_for_value[0]==0)
					{
					}

				if (wait_for_value[1]==2)
					{
					value_int[1][0]=4;
					wait_for_value[1]=1;
					}
				else if (wait_for_value[1]==1)
					{
					value_int[1][1]=4;
					wait_for_value[1]=0;
					}
				else if (wait_for_value[1]==0)
					{
					}

			}
		else if (mark == '5')
			{
				if (wait_for_value[0]==2)
					{
					value_int[0][0]=5;
					wait_for_value[0]=1;
					}
				else if (wait_for_value[0]==1)
					{
					value_int[0][1]=5;
					wait_for_value[0]=0;
					}
				else if (wait_for_value[0]==0)
					{
					}


				if (wait_for_value[1]==2)
					{
					value_int[1][0]=5;
					wait_for_value[1]=1;
					}
				else if (wait_for_value[1]==1)
					{
					value_int[1][1]=5;
					wait_for_value[1]=0;
					}
				else if (wait_for_value[1]==0)
					{
					}
			}

		else if (mark == '6')
			{
				if (wait_for_value[0]==2)
					{
					value_int[0][0]=6;
					wait_for_value[0]=1;
					}
				else if (wait_for_value[0]==1)
					{
					value_int[0][1]=6;
					wait_for_value[0]=0;
					}
				else if (wait_for_value[0]==0)
					{
					}


				if (wait_for_value[1]==2)
					{
					value_int[1][0]=6;
					wait_for_value[1]=1;
					}
				else if (wait_for_value[1]==1)
					{
					value_int[1][1]=6;
					wait_for_value[1]=0;
					}
				else if (wait_for_value[1]==0)
					{
					}
			}

		else if (mark == '7')
			{
				if (wait_for_value[0]==2)
					{
					value_int[0][0]=7;
					wait_for_value[0]=1;
					}
				else if (wait_for_value[0]==1)
					{
					value_int[0][1]=7;
					wait_for_value[0]=0;
					}
				else if (wait_for_value[0]==0)
					{
					}


				if (wait_for_value[1]==2)
					{
					value_int[1][0]=7;
					wait_for_value[1]=1;
					}
				else if (wait_for_value[1]==1)
					{
					value_int[1][1]=7;
					wait_for_value[1]=0;
					}
				else if (wait_for_value[1]==0)
					{
					}
			}

		else if (mark == '8')
			{
				if (wait_for_value[0]==2)
					{
					value_int[0][0]=8;
					wait_for_value[0]=1;
					}
				else if (wait_for_value[0]==1)
					{
					value_int[0][1]=8;
					wait_for_value[0]=0;
					}
				else if (wait_for_value[0]==0)
					{
					}


				if (wait_for_value[1]==2)
					{
					value_int[1][0]=8;
					wait_for_value[1]=1;
					}
				else if (wait_for_value[1]==1)
					{
					value_int[1][1]=8;
					wait_for_value[1]=0;
					}
				else if (wait_for_value[1]==0)
					{
					}
			}

		else if (mark == '9')
		{
				if (wait_for_value[0]==2)
					{
					value_int[0][0]=9;
					wait_for_value[0]=1;
					}
				else if (wait_for_value[0]==1)
					{
					value_int[0][1]=9;
					wait_for_value[0]=0;
					}
				else if (wait_for_value[0]==0)
					{
					}

				if (wait_for_value[1]==2)
					{
					value_int[1][0]=9;
					wait_for_value[1]=1;
					}
				else if (wait_for_value[1]==1)
					{
					value_int[1][1]=9;
					wait_for_value[1]=0;
					}
				else if (wait_for_value[1]==0)
					{
					}

		}
		}
		HAL_UART_Receive_IT(&huart2, &mark, 1);
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//Changing states with f/6 frequency
	if(htim->Instance == TIM1)
	{
		//Inverter can be controlled also with adc
				if (!diode_mode)
				{
			  HAL_ADC_Start(&hadc1);
			  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			  value_adc_read = HAL_ADC_GetValue(&hadc1);
			  HAL_ADC_Stop(&hadc1);
			  }

	}
	if(htim->Instance == TIM10)
	{
		state=state+1;
		if (state>=7) state=1;
		//Write_outputs(state);


	}
	//PWM with f/400 frequency
	if(htim->Instance == TIM11)
	{

		if (delay_phase< 300) delay_phase = delay_phase+1; //czekamy do 133 dla fazy 1 i 266 dla fazy 2
		i_1=i_1+1;
		if (i_1 > 20)
			{
			j_1=j_1+1;
			i_1=0;
			}
		if (j_1 > 20) j_1=0;
		if (i_1<=lookup[j_1])
		{
			Write_outputs_adv(state,1);
			debug=1;
		}
		else
		{
		HAL_GPIO_WritePin(GPIOC, 0b0000000000001001, GPIO_PIN_RESET);
		debug=0;
		}

		if (delay_phase>=133)
		{
			i_2=i_2+1;
			if (i_2 > 20)
				{
				j_2=j_2+1;
				i_2=0;
				}
			if (j_2 > 20) j_2=0;
			if (i_2<=lookup[j_2])
			{
				Write_outputs_adv(state,2);
			}
			else
			{
			HAL_GPIO_WritePin(GPIOC, 0b0000000000100100, GPIO_PIN_RESET);
			}
		}
		if (delay_phase>=266)
		{
			i_3=i_3+1;
			if (i_3 > 20)
				{
				j_3=j_3+1;
				i_3=0;
				}
			if (j_3 > 20) j_3=0;
			if (i_3<=lookup[j_3])
			{
				Write_outputs_adv(state,3);
			}
			else
			{
			HAL_GPIO_WritePin(GPIOC, 0b0000000000010010, GPIO_PIN_RESET);
			}
		}
	}

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//frequnecy set with adc is activated by Button on board (interrupt)
	if (!diode_mode)
					{
			  freq_wanted = (value_adc_read)*50/4096+1;
			  freq_actual=50/freq_wanted;
			  htim10.Instance = TIM10;
			  htim10.Init.Prescaler = 59999;
			  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
			  htim10.Init.Period = 5*freq_actual - 1;
			  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
			  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
			  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
			  {
				Error_Handler();
			  }
			  htim11.Instance = TIM11;
			  htim11.Init.Prescaler = 49;
			  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
			  htim11.Init.Period = 5*6*freq_actual - 1;
			  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
			  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
			  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
			  {
			    Error_Handler();
			  }
			  HAL_TIM_Base_Start_IT(&htim10);
			  HAL_TIM_Base_Start_IT(&htim11);
					}
}
void Write_outputs (int state)
{
			if (state == 0)
			{
				HAL_GPIO_WritePin(GPIOC, ~ST0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, ST0, GPIO_PIN_SET);
			}
			else if (state == 1)
			{
				HAL_GPIO_WritePin(GPIOC, ~ST1, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, ST1, GPIO_PIN_SET);
			}
			else if (state == 2)
			{
				HAL_GPIO_WritePin(GPIOC, ~ST2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, ST2, GPIO_PIN_SET);
			}
			else if (state == 3)
			{
				HAL_GPIO_WritePin(GPIOC, ~ST3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, ST3, GPIO_PIN_SET);
			}
			else if (state == 4)
			{
				HAL_GPIO_WritePin(GPIOC, ~ST4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, ST4, GPIO_PIN_SET);
			}
			else if (state == 5)
			{
				HAL_GPIO_WritePin(GPIOC, ~ST5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, ST5, GPIO_PIN_SET);
			}
			else if (state == 6)
			{
				HAL_GPIO_WritePin(GPIOC, ~ST6, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, ST6, GPIO_PIN_SET);
			}
}
void Write_outputs_adv (int state, int phase)
{

			if (phase == 1)
			{
				if (state == 0)
				{
					HAL_GPIO_WritePin(GPIOC,0b0000000000001001, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000000, GPIO_PIN_SET);
				}
				else if (state == 1)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000001000, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000001, GPIO_PIN_SET);
				}
				else if (state == 2)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000001000, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000001, GPIO_PIN_SET);
				}
				else if (state == 3)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000001000, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000001, GPIO_PIN_SET);
				}
				else if (state == 4)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000001, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000001000, GPIO_PIN_SET);
				}
				else if (state == 5)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000001, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000001000, GPIO_PIN_SET);
				}
				else if (state == 6)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000001, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000001000, GPIO_PIN_SET);
				}
			}
			if (phase == 2)
			{
				if (state == 0)
				{
					HAL_GPIO_WritePin(GPIOC,0b0000000000100100, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000000, GPIO_PIN_SET);
				}
				else if (state == 1)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000100, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000100000, GPIO_PIN_SET);
				}
				else if (state == 2)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000100, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000100000, GPIO_PIN_SET);
				}
				else if (state == 3)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000100000, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000100, GPIO_PIN_SET);
				}
				else if (state == 4)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000100000, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000100, GPIO_PIN_SET);
				}
				else if (state == 5)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000100000, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000100, GPIO_PIN_SET);
				}
				else if (state == 6)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000100, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000100000, GPIO_PIN_SET);
				}
			}
			if (phase == 3)
			{
				if (state == 0)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000010010, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000000, GPIO_PIN_SET);
				}
				else if (state == 1)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000010, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000010000, GPIO_PIN_SET);
				}
				else if (state == 2)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000010000, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000010, GPIO_PIN_SET);
				}
				else if (state == 3)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000010000, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000010, GPIO_PIN_SET);
				}
				else if (state == 4)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000010000, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000010, GPIO_PIN_SET);
				}
				else if (state == 5)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000010, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000010000, GPIO_PIN_SET);
				}
				else if (state == 6)
				{
					HAL_GPIO_WritePin(GPIOC, 0b0000000000000010, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 0b0000000000010000, GPIO_PIN_SET);
				}
			}
}
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
