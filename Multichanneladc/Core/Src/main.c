/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS_Pin GPIO_PIN_2
#define RS_GPIO_Port GPIOB
#define RW_Pin GPIO_PIN_3
#define RW_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_4
#define EN_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_5
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_6
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_7
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_8
#define D7_GPIO_Port GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

osThreadId PulseTaskHandle;
osThreadId TempTaskHandle;
osTimerId a_min_timerHandle;
/* USER CODE BEGIN PV */
uint16_t pul=0;
float tem=0;
uint16_t rawValues[2]={0,0};
char msg[60];
char msg_a[60];
char msg_b[60];
int BPM[1]={0};
float TEMP[1]={0};
int i=0;
int debug_mode=0;
int debug_start_screen=0;

uint16_t ADC1_value=0;
float ADC2_value=0;

uint16_t last_ADC1_value=0;
float last_ADC2_value=0;

//average temp reading.

int temp_sensor_raw_data[30];
int temp_loop=0;
float temp_average_raw=0;

// Custom Icons
char heart[] = {
		  0x00,
		  0x11,
		  0x1B,
		  0x1F,
		  0x1F,
		  0x0E,
		  0x04,
		  0x00
		};  // heart:0


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
void StartPulseTask(void const * argument);
void StartTempTask(void const * argument);
void Callback_a_min_timer(void const * argument);

/* USER CODE BEGIN PFP */
//LCD interface
void lcd_init (void);
void lcd_send_cmd (char cmd);
void lcd_send_data (char data);
void lcd_send_string (char *str);
void lcd_put_cur(int row, int col);
void lcd_clear (void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t convCompleted = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	convCompleted = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
	debug_mode=1;
	debug_start_screen=1;

  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  lcd_init();

  // store the chars into the CGRAM
  //lcd_send_cmd(0x40);
  //for (int i=0; i<8; i++) lcd_send_data(cc5[i]);

  lcd_clear();
  //lcd_put_cur(0,0);
  //lcd_send_data(heart); // displaying heart.
  lcd_put_cur(0,0);
  lcd_send_string(" Health Monitor");
  lcd_put_cur(1,0);
  lcd_send_string("     System");

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of a_min_timer */
  osTimerDef(a_min_timer, Callback_a_min_timer);
  a_min_timerHandle = osTimerCreate(osTimer(a_min_timer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of PulseTask */
  osThreadDef(PulseTask, StartPulseTask, osPriorityAboveNormal, 0, 128);
  PulseTaskHandle = osThreadCreate(osThread(PulseTask), NULL);

  /* definition and creation of TempTask */
  osThreadDef(TempTask, StartTempTask, osPriorityAboveNormal, 0, 128);
  TempTaskHandle = osThreadCreate(osThread(TempTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB3 PB4 PB5
                           PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// LED INTERFACE.

void delay (uint16_t us)


{
	int oo = 0;

	while (oo < us) {
		oo++;
}
	}


/******************************************************/

void send_to_lcd (char data, int rs)
{
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, rs);  // rs = 1 for data, rs=0 for command

	/* write the data to the respective pin */
	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, ((data>>3)&0x01));
	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, ((data>>2)&0x01));
	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, ((data>>1)&0x01));
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, ((data>>0)&0x01));


	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 0);
	HAL_Delay(1);
}

void lcd_send_cmd (char cmd)
{
    char datatosend;

    /* send upper nibble first */
    datatosend = ((cmd>>4)&0x0f);
    send_to_lcd(datatosend,0);  // RS must be 0 while sending command

    /* send Lower Nibble */
    datatosend = ((cmd)&0x0f);
	send_to_lcd(datatosend, 0);
}

void lcd_send_data (char data)
{
	char datatosend;

	/* send higher nibble */
	datatosend = ((data>>4)&0x0f);
	send_to_lcd(datatosend, 1);  // rs =1 for sending data

	/* send Lower nibble */
	datatosend = ((data)&0x0f);
	send_to_lcd(datatosend, 1);
}

void lcd_clear (void)
{
	lcd_send_cmd(0x01);
	HAL_Delay(1);
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}


void lcd_init (void)
{

	HAL_Delay(10);
	lcd_send_cmd (0x02);
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	lcd_send_cmd (0x01);  // clear display
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartPulseTask */
/**
  * @brief  Function implementing the PulseTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartPulseTask */
void StartPulseTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	//sprintf(msg,"Pulse task called\r\n");
	if(debug_mode==0){
	osTimerStart(a_min_timerHandle,10000);
	}
	else
	{
	osTimerStop(a_min_timerHandle);
	osTimerStart(a_min_timerHandle,20);
	}
  /* Infinite loop */
  for(;;)
  {
	 //sprintf(msg,"Pulse task called\r\n");
	 //HAL_UART_Transmit(&huart2,(uint8_t *)msg,strlen(msg),HAL_MAX_DELAY);

	 HAL_ADC_Start(&hadc1);
	 HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	 ADC1_value = HAL_ADC_GetValue(&hadc1);
	 HAL_ADC_Stop(&hadc1);

	 //sprintf(msg,"Pulse: %hu\r\n",ADC1_value);
	 //HAL_UART_Transmit(&huart2,(uint8_t *)msg,strlen(msg),HAL_MAX_DELAY);

	  if(ADC1_value>4000 && last_ADC1_value<4000)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  osDelay(50);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		  osDelay(50);
		  BPM[i]=BPM[i]+1;
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	  }

	  last_ADC1_value=ADC1_value;

     osDelay(300);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTempTask */
/**
* @brief Function implementing the TempTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTempTask */
void StartTempTask(void const * argument)
{
  /* USER CODE BEGIN StartTempTask */
  /* Infinite loop */
  for(;;)
  {
	  //sprintf(msg,"Temp task called\r\n");
	  	 //HAL_UART_Transmit(&huart2,(uint8_t *)msg,strlen(msg),HAL_MAX_DELAY);

	  	 while(temp_loop<30)
	  	 {
	  	 HAL_ADC_Start(&hadc2);
	  	 HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	  	 ADC2_value = HAL_ADC_GetValue(&hadc2);
	  	 HAL_ADC_Stop(&hadc2);
	  	 temp_sensor_raw_data[temp_loop]=ADC2_value;
	  	 temp_loop++;
	  	 }

	  	 temp_loop=0;

	  	 while(temp_loop<30)
	  	 {
	  	 temp_average_raw = temp_average_raw + temp_sensor_raw_data[temp_loop];
	  	 temp_loop++;
	  	 }

	  	 temp_average_raw=temp_average_raw/30;
	  	 temp_loop=0;
	     osDelay(1);
  }
  /* USER CODE END StartTempTask */
}

/* Callback_a_min_timer function */
void Callback_a_min_timer(void const * argument)
{
  /* USER CODE BEGIN Callback_a_min_timer */
	int final=0;
	int temp_final=0;

	// debug mode message start.
	if(debug_start_screen==1)
	{
		sprintf(msg_a,"Activating");
		sprintf(msg_b,"Debug Mode...");
		lcd_clear();
		lcd_put_cur(0,0);
	    lcd_send_string(msg_a);
	    lcd_put_cur(1,0);
	    lcd_send_string(msg_b);
	    debug_start_screen=0;
	    osDelay(2000);
	}
	else{
		// For future use.
	};

	// debug mode message end.

	//sprintf(msg,"Timer task working\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t *)msg,strlen(msg),HAL_MAX_DELAY);

	final = BPM[i]*6;

	TEMP[i]= (temp_average_raw/4095)*2.4;
	TEMP[i]= TEMP[i]*100;
	temp_final = (int)TEMP[i];

	if(final>59)
	{

		if (debug_mode==0)
		{
			sprintf(msg,"BPM: %d\r\n",final);
			HAL_UART_Transmit(&huart2,(uint8_t *)msg,strlen(msg),HAL_MAX_DELAY);

			sprintf(msg,"Temp: %d *C\r\n",temp_final);
			HAL_UART_Transmit(&huart2,msg,strlen(msg),HAL_MAX_DELAY);

			sprintf(msg_a,"BPM: %d",final);
			sprintf(msg_b,"Temp: %d C",temp_final);
			 lcd_clear();
			 lcd_put_cur(0,0);
			 lcd_send_string(msg_a);
			 lcd_put_cur(1,0);
			 lcd_send_string(msg_b);
		}
		else
		{
			sprintf(msg_a,"ADC1: %d",ADC1_value);
			sprintf(msg_b,"ADC2: %d",(int)ADC2_value);
			lcd_clear();
			lcd_put_cur(0,0);
			lcd_send_string(msg_a);
			lcd_put_cur(1,0);
			lcd_send_string(msg_b);
		}

	}


	else if(final<59 && final>10)
	{


		if (debug_mode==0)
		{
			sprintf(msg,"Recording data...\r\n");
			HAL_UART_Transmit(&huart2,(uint8_t *)msg,strlen(msg),HAL_MAX_DELAY);

			 lcd_clear();
			 lcd_put_cur(0,0);
			 lcd_send_string("Recording");
			 lcd_put_cur(1,0);
			 lcd_send_string("Data....");
		}
		else
		{
			sprintf(msg_a,"ADC1: %d",ADC1_value);
			sprintf(msg_b,"ADC2: %d",(int)ADC2_value);
			lcd_clear();
			lcd_put_cur(0,0);
			lcd_send_string(msg_a);
			lcd_put_cur(1,0);
			lcd_send_string(msg_b);
		}


	}
	else if (final<10)
	{

		if (debug_mode==0)
				{
			sprintf(msg,"Please put your finger on sensor\r\n");
			sprintf(msg_b,"Temp: %d C",temp_final);
			HAL_UART_Transmit(&huart2,(uint8_t *)msg,strlen(msg),HAL_MAX_DELAY);

			 lcd_clear();
			 lcd_put_cur(0,0);
			 lcd_send_string("BPM: No finger!"); //no_finger
			 lcd_put_cur(1,0);
			 lcd_send_string(msg_b);
				}
				else
				{
					sprintf(msg_a,"ADC1: %d",ADC1_value);
					sprintf(msg_b,"ADC2: %d",(int)ADC2_value);
					lcd_clear();
					lcd_put_cur(0,0);
					lcd_send_string(msg_a);
					lcd_put_cur(1,0);
					lcd_send_string(msg_b);
				}


	}
	//osThreadResume(PulseTaskHandle);
	BPM[i]=i;
  /* USER CODE END Callback_a_min_timer */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
