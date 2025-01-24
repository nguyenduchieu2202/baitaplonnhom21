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
#include "stm32f10x.h"
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "string.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
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


// Biến toàn cục
float temperature = 0.0, humidity = 0.0;
char uart_buffer[50];

// Queue để chia sẻ dữ liệu giữa task
QueueHandle_t SensorDataQueue;

// Hàm đọc cảm biến (giả lập)
void Read_DHT(float *temp, float *humid) {
}

// Task đọc dữ liệu cảm biến
void Task_ReadSensor(void *argument) {
    float temp, humid;

    while (1) {
        Read_DHT(&temp, &humid);

        // Gửi dữ liệu vào queue
        float data[2] = {temp, humid};
        xQueueSend(SensorDataQueue, &data, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(1000)); // Chạy mỗi 1 giây
    }
}

// Task gửi UART
void Task_UARTSend(void *argument) {
    float data[2];

    while (1) {
        // Nhận dữ liệu từ queue
        if (xQueueReceive(SensorDataQueue, &data, portMAX_DELAY)) {
            sprintf(uart_buffer, "Temp: %.1f°C, Humidity: %.1f%%\r\n", data[0], data[1]);
            HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        }
    }
}

// Task hiển thị LED
void Task_DisplayLED(void *argument) {
    float data[2];

    while (1) {
        // Nhận dữ liệu từ queue
        if (xQueueReceive(SensorDataQueue, &data, portMAX_DELAY)) {
            int temp_int = (int)data[0];
            int digit1 = temp_int / 10;
            int digit2 = temp_int % 10;

            // Hiển thị lên LED 7 thanh
            LED_DisplayDigit(1, digit1);
            HAL_Delay(5);
            LED_DisplayDigit(2, digit2);
            HAL_Delay(5);
        }
    }
}

// Main function
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    // Khởi tạo queue
    SensorDataQueue = xQueueCreate(10, sizeof(float) * 2);

    // Tạo tasks
    xTaskCreate(Task_ReadSensor, "ReadSensor", 128, NULL, 2, NULL);
    xTaskCreate(Task_UARTSend, "UARTSend", 128, NULL, 1, NULL);
    xTaskCreate(Task_DisplayLED, "DisplayLED", 128, NULL, 3, NULL);

    // Start scheduler
    vTaskStartScheduler();

    while (1);
}

void Delay1Ms(void);
void Delay_Ms(uint32_t u32DelayInMs);
void send(uint8_t u8Data);

void Delay1Ms(void)
{

	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < 1000) {
	}
}

void Delay_Ms(uint32_t u32DelayInMs)
{

	while (u32DelayInMs) {
		Delay1Ms();
		--u32DelayInMs;
	}
}

void send(uint8_t u8Data)
{
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		if (u8Data & 0x80) {
			GPIO_SetBits(GPIOA, GPIO_Pin_0);
			Delay_Ms(4);
			GPIO_ResetBits(GPIOA, GPIO_Pin_0);
			Delay_Ms(1);
		} else {
			GPIO_SetBits(GPIOA, GPIO_Pin_0);
			Delay_Ms(1);
			GPIO_ResetBits(GPIOA, GPIO_Pin_0);
			Delay_Ms(4);
		}
		u8Data <<= 1;
	}
}

int main(void)
{
	GPIO_InitTypeDef gpioInit;
	TIM_TimeBaseInitTypeDef timerInit;
	uint16_t u16Tim;
	uint8_t u8Buff[5];
	uint8_t u8CheckSum;
	uint8_t i;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	gpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInit.GPIO_Pin = GPIO_Pin_13;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOC, &gpioInit);

	gpioInit.GPIO_Mode = GPIO_Mode_Out_OD;
	gpioInit.GPIO_Pin = GPIO_Pin_12;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &gpioInit);

	GPIO_SetBits(GPIOB, GPIO_Pin_12);

	timerInit.TIM_CounterMode = TIM_CounterMode_Up;
	timerInit.TIM_Period = 0xFFFF;
	timerInit.TIM_Prescaler = 72 - 1;

	TIM_TimeBaseInit(TIM2, &timerInit);

	TIM_Cmd(TIM2, ENABLE);

	gpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInit.GPIO_Pin = GPIO_Pin_0;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &gpioInit);


	while (1) {
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		Delay_Ms(20);
		GPIO_SetBits(GPIOB, GPIO_Pin_12);

		/* cho chan PB12 len cao */
		TIM_SetCounter(TIM2, 0);
		while (TIM_GetCounter(TIM2) < 10) {
			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
				break;
			}
		}
		u16Tim = TIM_GetCounter(TIM2);
		if (u16Tim >= 10) {
			while (1) {
			}
		}

		/* cho chan PB12 xuong thap */
		TIM_SetCounter(TIM2, 0);
		while (TIM_GetCounter(TIM2) < 45) {
			if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
				break;
			}
		}
		u16Tim = TIM_GetCounter(TIM2);
		if ((u16Tim >= 45) || (u16Tim <= 5)) {
			while (1) {
			}
		}

		/* cho chan PB12 len cao */
		TIM_SetCounter(TIM2, 0);
		while (TIM_GetCounter(TIM2) < 90) {
			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
				break;
			}
		}
		u16Tim = TIM_GetCounter(TIM2);
		if ((u16Tim >= 90) || (u16Tim <= 70)) {
			while (1) {
			}
		}

		/* cho chan PB12 xuong thap */
		TIM_SetCounter(TIM2, 0);
		while (TIM_GetCounter(TIM2) < 95) {
			if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
				break;
			}
		}
		u16Tim = TIM_GetCounter(TIM2);
		if ((u16Tim >= 95) || (u16Tim <= 75)) {
			while (1) {
			}
		}

		/* nhan byte so 1 */
		for (i = 0; i < 8; ++i) {
			/* cho chan PB12 len cao */
			TIM_SetCounter(TIM2, 0);
			while (TIM_GetCounter(TIM2) < 65) {
				if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
					break;
				}
			}
			u16Tim = TIM_GetCounter(TIM2);
			if ((u16Tim >= 65) || (u16Tim <= 45)) {
				while (1) {
				}
			}

			/* cho chan PB12 xuong thap */
			TIM_SetCounter(TIM2, 0);
			while (TIM_GetCounter(TIM2) < 80) {
				if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
					break;
				}
			}
			u16Tim = TIM_GetCounter(TIM2);
			if ((u16Tim >= 80) || (u16Tim <= 10)) {
				while (1) {
				}
			}
			u8Buff[0] <<= 1;
			if (u16Tim > 45) {
				/* nhan duoc bit 1 */
				u8Buff[0] |= 1;
			} else {
				/* nhan duoc bit 0 */
				u8Buff[0] &= ~1;
			}
		}

		/* nhan byte so 2 */
		for (i = 0; i < 8; ++i) {
			/* cho chan PB12 len cao */
			TIM_SetCounter(TIM2, 0);
			while (TIM_GetCounter(TIM2) < 65) {
				if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
					break;
				}
			}
			u16Tim = TIM_GetCounter(TIM2);
			if ((u16Tim >= 65) || (u16Tim <= 45)) {
				while (1) {
				}
			}

			/* cho chan PB12 xuong thap */
			TIM_SetCounter(TIM2, 0);
			while (TIM_GetCounter(TIM2) < 80) {
				if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
					break;
				}
			}
			u16Tim = TIM_GetCounter(TIM2);
			if ((u16Tim >= 80) || (u16Tim <= 10)) {
				while (1) {
				}
			}
			u8Buff[1] <<= 1;
			if (u16Tim > 45) {
				/* nhan duoc bit 1 */
				u8Buff[1] |= 1;
			} else {
				/* nhan duoc bit 0 */
				u8Buff[1] &= ~1;
			}
		}

		/* nhan byte so 3 */
		for (i = 0; i < 8; ++i) {
			/* cho chan PB12 len cao */
			TIM_SetCounter(TIM2, 0);
			while (TIM_GetCounter(TIM2) < 65) {
				if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
					break;
				}
			}
			u16Tim = TIM_GetCounter(TIM2);
			if ((u16Tim >= 65) || (u16Tim <= 45)) {
				while (1) {
				}
			}

			/* cho chan PB12 xuong thap */
			TIM_SetCounter(TIM2, 0);
			while (TIM_GetCounter(TIM2) < 80) {
				if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
					break;
				}
			}
			u16Tim = TIM_GetCounter(TIM2);
			if ((u16Tim >= 80) || (u16Tim <= 10)) {
				while (1) {
				}
			}
			u8Buff[2] <<= 1;
			if (u16Tim > 45) {
				/* nhan duoc bit 1 */
				u8Buff[2] |= 1;
			} else {
				/* nhan duoc bit 0 */
				u8Buff[2] &= ~1;
			}
		}

		/* nhan byte so 4 */
		for (i = 0; i < 8; ++i) {
			/* cho chan PB12 len cao */
			TIM_SetCounter(TIM2, 0);
			while (TIM_GetCounter(TIM2) < 65) {
				if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
					break;
				}
			}
			u16Tim = TIM_GetCounter(TIM2);
			if ((u16Tim >= 65) || (u16Tim <= 45)) {
				while (1) {
				}
			}

			/* cho chan PB12 xuong thap */
			TIM_SetCounter(TIM2, 0);
			while (TIM_GetCounter(TIM2) < 80) {
				if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
					break;
				}
			}
			u16Tim = TIM_GetCounter(TIM2);
			if ((u16Tim >= 80) || (u16Tim <= 10)) {
				while (1) {
				}
			}
			u8Buff[3] <<= 1;
			if (u16Tim > 45) {
				/* nhan duoc bit 1 */
				u8Buff[3] |= 1;
			} else {
				/* nhan duoc bit 0 */
				u8Buff[3] &= ~1;
			}
		}

		/* nhan byte so 5 */
		for (i = 0; i < 8; ++i) {
			/* cho chan PB12 len cao */
			TIM_SetCounter(TIM2, 0);
			while (TIM_GetCounter(TIM2) < 65) {
				if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
					break;
				}
			}
			u16Tim = TIM_GetCounter(TIM2);
			if ((u16Tim >= 65) || (u16Tim <= 45)) {
				while (1) {
				}
			}

			/* cho chan PB12 xuong thap */
			TIM_SetCounter(TIM2, 0);
			while (TIM_GetCounter(TIM2) < 80) {
				if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
					break;
				}
			}
			u16Tim = TIM_GetCounter(TIM2);
			if ((u16Tim >= 80) || (u16Tim <= 10)) {
				while (1) {
				}
			}
			u8Buff[4] <<= 1;
			if (u16Tim > 45) {
				/* nhan duoc bit 1 */
				u8Buff[4] |= 1;
			} else {
				/* nhan duoc bit 0 */
				u8Buff[4] &= ~1;
			}
		}

		u8CheckSum = u8Buff[0] + u8Buff[1] + u8Buff[2] + u8Buff[3];
		if (u8CheckSum != u8Buff[4]) {
			while (1) {
			}
		}

		/* da doc duoc nhiet do va do am */
		/* gui do am */

		send(u8Buff[0]);

		//Delay_Ms(20);

		/* gui nhiet do */
		send(u8Buff[2]);

		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		Delay_Ms(500);
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		Delay_Ms(500);
	}

}

#endif
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
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
