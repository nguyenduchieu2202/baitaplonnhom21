/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 #define DHT11_PORT  GPIOA
 #define DHT11_PIN   GPIO_PIN_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

osThreadId Task1_ReadTempHandle;
osThreadId Task2_ReadHumiHandle;
osThreadId Task3_SendDataHandle;
osThreadId Task4_7displayHandle;
osMutexId R1_MutexHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
void Task1_GetTemperature(void const * argument);
void Task2_GetHumidity(void const * argument);
void Task3_SendTempHumidity(void const * argument);
void Task4_DisplayTempHumidity(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t digit_codes[10] = {
    0xC0, // 0
    0xF9, // 1
    0xA4, // 2
    0xB0, // 3
    0x99, // 4
    0x92, // 5
    0x82, // 6
    0xF8, // 7
    0x80, // 8
    0x90  // 9
};

uint8_t dp[10] = {
    0x20, // 0
    0x79, // 1
    0x24, // 2
    0x30, // 3
    0x19, // 4
    0x12, // 5
    0x02, // 6
    0x78, // 7
    0x00, // 8
    0x10  // 9
};

void ConvertNumberToDigits(uint16_t number, uint8_t *digits){
    digits[3] = digit_codes[number % 10];
    digits[2] = digit_codes[(number / 10) % 10];
    digits[1] = dp[(number / 100) % 10];
    digits[0] = digit_codes[(number / 1000) % 10];
}

void SendToLED_SPI(uint8_t* digits){
    // LOAD xuống thấp trước khi gửi dữ liệu
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    // Gửi lần lượt 4 byte (4 chữ số) qua SPI
    for (int i = 0; i < 4; i++){
        HAL_SPI_Transmit(&hspi1, &digits[3 - i], 1, HAL_MAX_DELAY); // Gửi byte từ trái sang phải
    }

    // LOAD lên cao để chốt dữ liệu hiển thị
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void delay(uint16_t time){
	 __HAL_TIM_SET_COUNTER(&htim1,0);
	 while((__HAL_TIM_GET_COUNTER(&htim1)) < time);
 }

 uint8_t R1, R2, T1, T2;
 uint16_t sum;
 float RH, TEMP;
 float Temperature = 0;
 float Humidity = 0;
 uint8_t Presence = 0;

 void SET_PIN_OUTPUT (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 }

 void SET_PIN_INPUT (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 }

 void DHT11_Start (void){
 	SET_PIN_OUTPUT(DHT11_PORT, DHT11_PIN);
 	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
 	delay(18000);   // wait for 18ms
 	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);
 	delay(20);
 	SET_PIN_INPUT(DHT11_PORT, DHT11_PIN);
 }

  uint8_t Check_Response (void){
 	uint8_t Response = 0;
 	delay (40);
 	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))){
 		delay (80);
 		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
 		else Response = -1;  //255
 	}
 	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // doi pin xung thap

 	return Response;
 }

 uint8_t DHT11_Read (void){
 	uint8_t i, j;
 	for (j=0; j<8; j++){
 		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));
 		delay (40);   // wait for 40 us, cho trong 40us, neu Readdht11 ma xuong 0 --> nhan bit 0, neu sau 40us ma chua xuong 0--> nhan bit 1
 		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
 		{
 			i&= ~(1<<(7-j));   // write 0
 		}
 		else {
 			i|= (1<<(7-j));
 		}			// pin high , write 1
 		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));
 	}
 	return i;
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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //osKernelInitialize();
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of R1_Mutex */
  osMutexDef(R1_Mutex);
  R1_MutexHandle = osMutexCreate(osMutex(R1_Mutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* Create Mutex */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task1_ReadTemp */
  osThreadDef(Task1_ReadTemp, Task1_GetTemperature, osPriorityNormal, 0, 128);
  Task1_ReadTempHandle = osThreadCreate(osThread(Task1_ReadTemp), NULL);

  /* definition and creation of Task2_ReadHumi */
  osThreadDef(Task2_ReadHumi, Task2_GetHumidity, osPriorityNormal, 0, 128);
  Task2_ReadHumiHandle = osThreadCreate(osThread(Task2_ReadHumi), NULL);

  /* definition and creation of Task3_SendData */
  osThreadDef(Task3_SendData, Task3_SendTempHumidity, osPriorityNormal, 0, 128);
  Task3_SendDataHandle = osThreadCreate(osThread(Task3_SendData), NULL);

  /* definition and creation of Task4_7display */
  osThreadDef(Task4_7display, Task4_DisplayTempHumidity, osPriorityNormal, 0, 128);
  Task4_7displayHandle = osThreadCreate(osThread(Task4_7display), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //osKernelInitialize();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 15;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task1_GetTemperature */
/**
  * @brief  Function implementing the Task1_ReadTemp thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task1_GetTemperature */
void Task1_GetTemperature(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	    char msg[] = "Task 1 is running\r\n";
	    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	    DHT11_Start();
	    Presence = Check_Response();
	    // Bảo vệ việc ghi vào R1
	    //osMutexWait(R1_MutexHandle, osWaitForever);
	    R1 = DHT11_Read();  // Ghi giá trị vào R1
	    //osMutexRelease(R1_MutexHandle);
	    //R1 = DHT11_Read();
	    R2 = DHT11_Read();
	    T1 = DHT11_Read();
	    T2 = DHT11_Read();
	    sum = DHT11_Read();
	    TEMP = T1 + T2 / 10.0;
	    Temperature = TEMP;
	    Humidity = R1;
	    //RH = R1;
	    osDelay(2000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task2_GetHumidity */
/**
* @brief Function implementing the Task2_ReadHumi thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task2_GetHumidity */
void Task2_GetHumidity(void const * argument)
{
  /* USER CODE BEGIN Task2_GetHumidity */
  /* Infinite loop */
  for(;;)
  {
	    char msg[] = "Task 2 is running\r\n";
	    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	    // Bảo vệ việc đ�?c từ R1
	    //osMutexWait(R1_MutexHandle, osWaitForever);
	    //RH = R1;  // �?�?c giá trị từ R1
	    //osMutexRelease(R1_MutexHandle);
	    //Humidity = RH;
	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // �?ổi trạng thái chân GPIO (nháy LED)
	    osDelay(2000);
  }
  /* USER CODE END Task2_GetHumidity */
}

/* USER CODE BEGIN Header_Task3_SendTempHumidity */
/**
* @brief Function implementing the Task3_SendData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task3_SendTempHumidity */
void Task3_SendTempHumidity(void const * argument)
{
  /* USER CODE BEGIN Task3_SendTempHumidity */
  /* Infinite loop */
  char buffer[50];
  for(;;)
  {
	    char msg[] = "Task 3 is running\r\n";
	    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	    snprintf(buffer, sizeof(buffer), "Temp: %.1f \n", Temperature);
	    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
	    snprintf(buffer, sizeof(buffer), "Hum: %.1f \n", Humidity);
	    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
	    osDelay(2000);
  }
  /* USER CODE END Task3_SendTempHumidity */
}

/* USER CODE BEGIN Header_Task4_DisplayTempHumidity */
/**
* @brief Function implementing the Task4_7display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task4_DisplayTempHumidity */
void Task4_DisplayTempHumidity(void const * argument)
{
  /* USER CODE BEGIN Task4_DisplayTempHumidity */
  /* Infinite loop */
  uint16_t temp_humi_display;
  uint8_t digits[4];
  for(;;)
  {
	    char msg[] = "Task 4 is running\r\n";
	    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	    temp_humi_display = (uint8_t)Temperature * 100 + (uint8_t)Humidity;
	    ConvertNumberToDigits(temp_humi_display, digits);
	    SendToLED_SPI(digits);
	    osDelay(2000);
  }
  /* USER CODE END Task4_DisplayTempHumidity */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
