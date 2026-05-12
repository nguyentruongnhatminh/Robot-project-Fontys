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
#include "robotProject.h"

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SysTick_Init(uint32_t ticks) {
	SysTick->CTRL = 0; //disable clock for configuration
	SysTick->LOAD = ticks - 1;

	NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1); // Set interrupt priority of SysTick to least urgency (i.e., largest priority value)

	SysTick->VAL = 0; // Reset the SysTick counter value
	SysTick->CTRL = 0x7; //setting the Enable, interrupt and clock source bit
}

void SysTick_Handler(void)
{
	timeCounter++; // TimeDelay is a global volatile variable
}

//configure the GPIO pins
void GPIO_port_configuration(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // configure clock port A
	// servo1 I'm using PA10
	GPIOA->AFR[1] |= (1 << 9) | (1 << 11);//TIM2 channel 4 used PA10 and AF10, servo 1
	GPIOA->AFR[0] |= (1 << 4); //TIM2 channel 2 used PBA1 and AF1, servo2
	GPIOA->MODER &= ~(3U << 20);
	GPIOA->MODER &= ~(3U << 2);
	GPIOA->MODER |= GPIO_MODER_MODER10_1; // set moder for servo1 (alternate function)
	GPIOA->MODER |= GPIO_MODER_MODER1_1; // set moder for servo2 (alternate function)

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // configure clock port B
	/*Input GPIO configuration*/
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL6;
	GPIOB->AFR[0] |= 0x2 << GPIO_AFRL_AFRL6_Pos; //TIM4 channel 1 used PB6 and AF2
	GPIOB->MODER &= ~GPIO_MODER_MODER6;
	GPIOB->MODER |= GPIO_MODER_MODER6_1; //alternate fuction
	GPIOB->PUPDR &= ~(3U << 12); // no PU/PD, dependent on the circuit

	GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL7;
	GPIOB->AFR[0] |= 0x2 << GPIO_AFRL_AFRL7_Pos; //TIM4 channel 1 used PB6 and AF2
	GPIOB->MODER &= ~GPIO_MODER_MODER7;
	GPIOB->MODER |= GPIO_MODER_MODER7_1; //alternate fuction
	GPIOB->PUPDR &= ~(3U << 14); // no PU/PD, dependent on the circuit
	/*Output GPIO configuration*/
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL5;
	GPIOB->AFR[0] |= 0x2 << GPIO_AFRL_AFRL5_Pos; //TIM3 channel 2 used PB5 and AF2
	GPIOB->MODER &= ~GPIO_MODER_MODER5;
	GPIOB->MODER |= GPIO_MODER_MODER5_1;
	/*LED PB4*/
	GPIOB->MODER &= ~GPIO_MODER_MODER4; //clear the moder register
	GPIOB->MODER |= GPIO_MODER_MODER4_0; //Set pins PB4 as output (LED)
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_4; //Set output push-pull
}

//configure the Timer register for the wheel
void Timer_configuration_servo(uint32_t PSC, uint32_t ARR) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable the timer 3 clock

	TIM2->PSC = PSC - 1; //set the prescaler, PWM frequency = 72kHz
	TIM2->ARR = ARR - 1; //PWM duty cycle
	TIM2->CNT = 0; // reset value

	TIM2->CCMR2 |= TIM_CCMR2_OC4PE; // enable output compare preload timer 2 (channel 4)
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE; // enable output compare preload timer 2 (channel 2)
	TIM2->CCMR2 |= (1 << 13) | (1 << 14); // set PWM mode to mode 1 on timer 2 (channel 4)
	TIM2->CCMR1 |= (1 << 13) | (1 << 14); // set PWM mode to mode 1 on timer 2 (channel 2)
	TIM2->CCER |= TIM_CCER_CC4E; //enable compare output channel 4
	TIM2->CCER |= TIM_CCER_CC2E; //enable compare output channel 2
	TIM2->CR1 = TIM_CR1_CEN; //enable counter timer 2
}

//configure the Timer register for the distance sensor
void Timer_configuration_ultrasonic(uint32_t PSC, uint32_t ARR) {
	/*Output timer configuration*/
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable the timer 3 clock
	TIM3->PSC = PSC - 1; //set the prescaler, PWM frequency = 72kHz
	TIM3->ARR = ARR - 1; //PWM duty cycle
	TIM3->CNT = 0; // reset value
	TIM3->CCR2 = 10; //the sensor must be receive a pulse of at lease 10us, in this case it is approximately 12us

	TIM3->CCMR1 |= TIM_CCMR1_OC2PE; // enable output compare preload timer 3 (channel 2)
	TIM3->CCMR1 &= ~TIM_CCMR1_OC2M_0;
	TIM3->CCMR1 &= ~TIM_CCMR1_OC2M_3;
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // set PWM mode to mode 1 on timer 3 (channel 2)
	TIM3->CCER |= TIM_CCER_CC2E; //enable compare output channel 2
	TIM3->CR1 = TIM_CR1_CEN; //enable counter timer 2

	/*Input timer configuration*/
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // enable the timer 4 clock

	TIM4->PSC = PSC - 1; //set the prescaler, PWM frequency = 72kHz
	TIM4->ARR = ARR - 1; //PWM duty cycle
	TIM4->CNT = 0; // reset value

	TIM4->CCMR1 &= ~TIM_CCMR1_IC1F; // clear the bits so the input filter durtion is 0
	TIM4->CCMR1 &= ~TIM_CCMR1_IC1PSC; //no PSC
	TIM4->CCMR1 &= ~TIM_CCMR1_CC1S_0; //configured CC1 on TI1
	TIM4->CCMR1 |= TIM_CCMR1_CC1S_1;
	TIM4->CCER &= ~TIM_CCER_CC1P; //non-inverted/ rising-edge
	TIM4->CCER &= ~TIM_CCER_CC1NP;
	TIM4->CCER |= TIM_CCER_CC1E; //enable capture channel
	TIM4->DIER |= TIM_DIER_CC1IE;

	TIM4->CCMR1 &= ~TIM_CCMR1_IC2F; // clear the bits so the input filter durtion is 0
	TIM4->CCMR1 &= ~TIM_CCMR1_IC2PSC; //no PSC
	TIM4->CCMR1 |= TIM_CCMR1_CC2S_0; //configured CC2 on TI1
	TIM4->CCMR1 &= ~TIM_CCMR1_CC2S_1;
	TIM4->CCER |= TIM_CCER_CC2P; //inverted/ falling-edge
	TIM4->CCER &= ~TIM_CCER_CC2NP;
	TIM4->CCER |= TIM_CCER_CC2E; //enable capture channel
	TIM4->DIER |= TIM_DIER_CC2IE;

	TIM4->SMCR &= ~TIM_SMCR_TS_1;
	TIM4->SMCR |= (TIM_SMCR_TS_0 | TIM_SMCR_TS_2);
	TIM4->SMCR |= TIM_SMCR_SMS_2;

	TIM4->CR1 = TIM_CR1_CEN; //enable counter timer 2
}
/* USER CODE END 0 */

//fuction to map the speed of the wheel
double MAP(double OldValue, double OldMin, double OldMax, double NewMin, double NewMax)
{
    double OldRange = (OldMax - OldMin);
    double NewRange = (NewMax - NewMin);
    double NewValue = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin;
    return NewValue;
}

void Generate_Turning_Decision(int inputNumber)
{
	timeCounter2 = 0;
	if(inputNumber == 0)
	{
		TIM2->CCR2 = CCRx_90_SERVO1; //moving forward
		TIM2->CCR4 = CCRx_MINUS_90_SERVO2; //moving back
		if (timeCounter2 >= 5000) //1s delay
		{
			timeCounter2 = 0;
		}
	}
	else if (inputNumber == 1)
	{
		TIM2->CCR2 = CCRx_MINUS_90_SERVO1; //moving back
		TIM2->CCR4 = CCRx_0_SERVO2; //moving forward
		if (timeCounter2 >= 5000) //1s delay
		{
			timeCounter2 = 0;
		}
	}
}

void adjust_robot_state(Robot_States state, int controlValue, int range)
{
	switch(state)
	{
		case MOVING_FOWARD:
			control_servo(controlValue);
			if(range >= -4)
			{
				timeCounter2 = 0;
//				unsigned int number = rand() % 2;
//				Generate_Turning_Decision(number);
				TIM2->CCR2 = CCRx_90_SERVO1; //moving forward
				TIM2->CCR4 = CCRx_MINUS_90_SERVO2; //moving back
				if (timeCounter2 >= 1000) //1s delay
				{
					timeCounter2 = 0;
				}
			}
			break;
		case MOVING_BACKWARD:
			control_servo(controlValue);
			if(range <= 4)
			{
				timeCounter2 = 0;
				TIM2->CCR2 = CCRx_90_SERVO1; //moving forward
				TIM2->CCR4 = CCRx_MINUS_90_SERVO2; //moving back
				if (timeCounter2 >= 1000) //1s delay
				{
					timeCounter2 = 0;
				}
			}
			break;
		default:
			break;
	}
}

void Adjust_Robot_By_Distance(int controlValue, int range)
{
	if(controlValue < MIN_SPEED_LIMIT)
	{
		controlValue = MIN_SPEED_LIMIT;
	}
	else if (controlValue > MAX_SPEED_LIMIT)
	{
		controlValue = MAX_SPEED_LIMIT;
	}

	if(range < 0)
	{
		adjust_robot_state(MOVING_FOWARD, controlValue, range);
	}
	else if(range > 0)
	{
		adjust_robot_state(MOVING_BACKWARD, controlValue, range);
	}
}

//function to genera the servo power
void control_servo(int controlValue)
{
	TIM2->CCR2 = MAP(controlValue, MIN_SPEED_LIMIT, MAX_SPEED_LIMIT, CCRx_MINUS_90_SERVO2, CCRx_90_SERVO2);
	TIM2->CCR4 = MAP(controlValue, MIN_SPEED_LIMIT, MAX_SPEED_LIMIT, CCRx_MINUS_90_SERVO1, CCRx_90_SERVO1);
}

int execute_PD(int error, int derivative, int KP, int KD)
{
	return (error*KP + derivative*KD);
}

//function to get the sensor value
float get_sensor_value(void)
{
	risingEdgeEvent = (TIM4->CCR1) / 58;
	fallingEdgeEvent = (TIM4->CCR2) / 58;
	distance = fallingEdgeEvent - risingEdgeEvent;
	return distance;
}

void my_PD(int setPoint) {
	timeCounter = 0; // timerCounter is a volatile global variable
	int error = 0;
	float sensorValue = 0;
	float previousError = 0;
	float derivative = 0;
	float KP = 5;
	float KD = 1.5;
	int power = 0;
	float delayMs = 20; //flag, servo is control every 20ms
	while (1) {
		if (timeCounter >= delayMs) //every 20ms period
		{
			sensorValue = get_sensor_value();
			error = setPoint - sensorValue; //updating the error
			derivative = error - previousError; //updating the derivative
			power = execute_PD(error, derivative, KP, KD);
			previousError = error;//updating the last error
			Adjust_Robot_By_Distance(power, error);
			timeCounter = 0; //updating the delay, possibility to overflow
		}
	}
}

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
	srand((unsigned int)time(0));
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	Timer_configuration_servo(16, 20000);
	Timer_configuration_ultrasonic(16, 0xFFFF);
	GPIO_port_configuration();
	SysTick_Init(16000);
	my_PD(30);
	while (1) {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 38400;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
	while (1) {
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
