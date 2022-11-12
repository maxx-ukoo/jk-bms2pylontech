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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "jk_bms_485.h"
#include "jk_bms_pylon.h"
#include "pylon_485.h"

#ifdef ENABLE_LCD

#include "ILI9341_GFX.h"
#include "ILI9341_STM32_Driver.h"

#endif /* ENABLE_LCD */


#define	__ENABLE_CONSOLE_DEBUG__	1
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef __ENABLE_CONSOLE_DEBUG__
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif /* __ENABLE_CONSOLE_DEBUG__ */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for getBMSDataTask */
osThreadId_t getBMSDataTaskHandle;
const osThreadAttr_t getBMSDataTask_attributes = {
  .name = "getBMSDataTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for every10msTask */
osThreadId_t every10msTaskHandle;
const osThreadAttr_t every10msTask_attributes = {
  .name = "every10msTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for consoleOutputTa */
osThreadId_t consoleOutputTaHandle;
const osThreadAttr_t consoleOutputTa_attributes = {
  .name = "consoleOutputTa",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for process_485_Request_Task */
osThreadId_t process_485_Request_TaskHandle;
const osThreadAttr_t process_485_Request_Task_attributes = {
  .name = "process_485_Request_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for consoleOutputQueue */
osMessageQueueId_t consoleOutputQueueHandle;
const osMessageQueueAttr_t consoleOutputQueue_attributes = {
  .name = "consoleOutputQueue"
};
/* USER CODE BEGIN PV */
uint8_t             UART_Rx_RS485_PYLON_Buffer[64];
uint8_t				rs485_Pylon_data_size = 0;
uint8_t				rs485_Pylon_data_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
void startGetBMSDataTask(void *argument);
void startEvery10msTask(void *argument);
void startConsoleOutputTask(void *argument);
void startProcess_485_Request_Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t				USB_Console_TX_Buffer[256];
uint8_t				USB_Console_TX_Buffer_Count = 0;
uint8_t             UART_Rx_Buffer[1024];
uint16_t			UART_Rx_Size;
uint16_t			UART_Rx_Current_Size;
uint8_t				dataReady = 5;
bms_type			protocol_type = v0;
#ifdef ENABLE_LCD
SPI_HandleTypeDef	lcd_spi;

uint8_t				lcd_soc = 110;
uint16_t			lcd_alarms = 523;
uint16_t			lcd_temp;

#endif /* ENABLE_LCD */
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of consoleOutputQueue */
  consoleOutputQueueHandle = osMessageQueueNew (32, sizeof(uint8_t), &consoleOutputQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of getBMSDataTask */
  getBMSDataTaskHandle = osThreadNew(startGetBMSDataTask, NULL, &getBMSDataTask_attributes);

  /* creation of every10msTask */
  every10msTaskHandle = osThreadNew(startEvery10msTask, NULL, &every10msTask_attributes);

  /* creation of consoleOutputTa */
  consoleOutputTaHandle = osThreadNew(startConsoleOutputTask, NULL, &consoleOutputTa_attributes);

  /* creation of process_485_Request_Task */
  process_485_Request_TaskHandle = osThreadNew(startProcess_485_Request_Task, NULL, &process_485_Request_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
#ifdef ENABLE_LCD

  ILI9341_Init();
  //ILI9341_Set_Rotation(SCREEN_VERTICAL_3); // Bohdan work
  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
  ILI9341_Fill_Screen(WHITE);

#endif /* ENABLE_LCD */

  //ILI9341_Init();
  //ILI9341_FillScreen(ILI9341_BLACK);
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
#ifdef ENABLE_LCD

  lcd_spi = hspi2;

#endif


  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  uint32_t port_rate = 115200;
  if (!HAL_GPIO_ReadPin (BMS_TYPE_SWITCH_GPIO_Port, BMS_TYPE_SWITCH_Pin)) {
	  port_rate = 9600;
	  protocol_type = v1;
  }
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = port_rate;
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
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, UART_Rx_Buffer, 350);
  /* USER CODE END USART1_Init 2 */

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
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, UART_Rx_RS485_PYLON_Buffer, 32);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TFT_RST_Pin|TFT_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_D2_Pin */
  GPIO_InitStruct.Pin = LED_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TFT_DC_Pin */
  GPIO_InitStruct.Pin = TFT_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TFT_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_RST_Pin TFT_CS_Pin */
  GPIO_InitStruct.Pin = TFT_RST_Pin|TFT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BMS_TYPE_SWITCH_Pin */
  GPIO_InitStruct.Pin = BMS_TYPE_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BMS_TYPE_SWITCH_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

#ifdef __ENABLE_CONSOLE_DEBUG__
PUTCHAR_PROTOTYPE
{
	//HAL_GPIO_TogglePin(LED_D2_GPIO_Port, LED_D2_Pin); //Toggle the state of pin
	USB_Console_TX_Buffer[USB_Console_TX_Buffer_Count] = ch;
	USB_Console_TX_Buffer_Count += 1;
	if (USB_Console_TX_Buffer_Count > 128) {
		taskENTER_CRITICAL();
		CDC_Transmit_FS(USB_Console_TX_Buffer, USB_Console_TX_Buffer_Count);
		USB_Console_TX_Buffer_Count = 0;
		taskEXIT_CRITICAL();
	}

	return ch;
}
#endif /* __ENABLE_CONSOLE_DEBUG__ */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
	if (huart->Instance == USART1) {
		UART_Rx_Size += size;
		if (JK_Battery_485_Check_Frame_CRC(UART_Rx_Buffer, UART_Rx_Size, protocol_type) == true) {
			UART_Rx_Current_Size = 10;
		} else {
			HAL_UARTEx_ReceiveToIdle_IT(huart, &UART_Rx_Buffer[UART_Rx_Size], 350);
		}
	} else if (huart->Instance == USART2) {
		rs485_Pylon_data_size += size;
		if (Pylon_485_Check_CRC(UART_Rx_RS485_PYLON_Buffer, rs485_Pylon_data_size)) {
			rs485_Pylon_data_ready = 1;
		} else {
			HAL_UARTEx_ReceiveToIdle_IT(huart, &UART_Rx_RS485_PYLON_Buffer[rs485_Pylon_data_size], 32);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		printf("HAL_UART_RxCpltCallback USART1\r\n");
	} else if (huart->Instance == USART2) {
		printf("HAL_UART_RxCpltCallback USART2\r\n");
	}
}

#ifdef ENABLE_LCD

void drawBattery(uint8_t soc) {
	if (lcd_soc != soc) {
		uint8_t x = 5;
		uint8_t yStart = 10;

		uint8_t width = 60;
		uint8_t step_height = 25;

		ILI9341_Draw_Filled_Rectangle_Coord(x + width/2 - 10, yStart, x + width/2 + 10, yStart - 5, BLACK);
		ILI9341_Draw_Filled_Rectangle_Coord(x, yStart, x + width, yStart + step_height * 5, BLACK);

		uint16_t state_color = GREEN;
		if (soc < 40) {
			state_color = ORANGE;
		}
		if (soc < 20) {
			state_color = RED;
		}

		for (int j=0; j<5; j++) {
			uint8_t current_soc = 20 * j;
			uint8_t i = 5 - j;
			if (soc > current_soc) {
				ILI9341_Draw_Filled_Rectangle_Coord(x + 4, yStart + step_height * (i - 1) + 4, x + width - 4, yStart + step_height * i - 4 , state_color);
			}
		}
		lcd_soc = soc;
	}
}

void drawVoltage_Current() {
	char str[32];
	uint16_t voltage = jk_bms_battery_info.battery_status.battery_voltage;
	int16_t current = jk_bms_battery_info.battery_status.battery_current;
	if (current > 0) {
		// charging
		ILI9341_Draw_Text_Space(">", 85, 18, WHITE, Font_26_B, WHITE, 15);
		ILI9341_Draw_Text_Space("<", 65, 18, BLACK, Font_26_B, WHITE, 15);
		ILI9341_Draw_Filled_Rectangle_Coord(70, 32, 105, 37, BLACK);
	} else {
		// discharging
		ILI9341_Draw_Text_Space("<", 65, 18, WHITE, Font_26_B, WHITE, 15);
		ILI9341_Draw_Text_Space(">", 85, 18, BLACK, Font_26_B, WHITE, 15);
		ILI9341_Draw_Filled_Rectangle_Coord(70, 32, 105, 37, BLACK);
	}

	if (current < 0) {
		current = -current;
	}


	sprintf(str, "%d", (int)(voltage/100));

	ILI9341_Draw_Text(str, 112, 10, DARKCYAN, Font_37x47, WHITE);
	ILI9341_Draw_Text(".", 190, 26, DARKCYAN, Font_34x36, WHITE);

	sprintf(str, "%02d", (int)(voltage%100));
	ILI9341_Draw_Text(str, 200, 10, DARKCYAN, Font_37x47, WHITE);

	ILI9341_Draw_Text("V", 275, 26, DARKCYAN, Font_34x36, WHITE);

	sprintf(str, "%03d", (int)(current/100));

	ILI9341_Draw_Text(str, 75, 70, DARKCYAN, Font_37x47, WHITE);
	ILI9341_Draw_Text(".", 190, 86, DARKCYAN, Font_34x36, WHITE);


	sprintf(str, "%02d", (int)(current%100));

	ILI9341_Draw_Text(str, 200, 70, DARKCYAN, Font_37x47, WHITE);

	ILI9341_Draw_Text("A", 275, 86, DARKCYAN, Font_34x36, WHITE);
}

void drawTemperature() {
	char str[32];
	sprintf(str, "%03dC", jk_bms_battery_info.battery_status.sensor_temperature_1);
	ILI9341_Draw_Text_Space(str, 80, 120, MAROON, Font_26_B, WHITE, 25);

	sprintf(str, "%03dC", jk_bms_battery_info.battery_status.sensor_temperature_2);
	ILI9341_Draw_Text_Space(str, 200, 120, MAROON, Font_26_B, WHITE, 25);
}

void drawAlarms() {
	uint16_t color = LIGHTGREY;

	if (jk_bms_battery_info.battery_alarms.charging_overvoltage) {
		color = RED;
	} else {
		color = LIGHTGREY;
	}
	ILI9341_Draw_Text_Space("COVT", 5, 200, color, Font_26_B, WHITE, 25);
	if (jk_bms_battery_info.battery_alarms.discharging_undervoltage) {
		color = RED;
	} else {
		color = LIGHTGREY;
	}
	ILI9341_Draw_Text_Space("DUVT", 110, 200, color, Font_26_B, WHITE, 25);

	if (jk_bms_battery_info.battery_alarms.low_capacity) {
		color = RED;
	} else {
		color = LIGHTGREY;
	}
	ILI9341_Draw_Text_Space("LCAP", 215, 200, color, Font_26_B, WHITE, 25);

	if (jk_bms_battery_info.battery_alarms.charging_overcurrent) {
		color = RED;
	} else {
		color = LIGHTGREY;
	}
	ILI9341_Draw_Text_Space("COVC", 5, 160, color, Font_26_B, WHITE, 25);

	if (jk_bms_battery_info.battery_alarms.discharging_overcurrent) {
		color = RED;
	} else {
		color = LIGHTGREY;
	}
	ILI9341_Draw_Text_Space("DOVC", 110, 160, color, Font_26_B, WHITE, 25);

	if (jk_bms_battery_info.battery_alarms.battery_over_temperature) {
		color = RED;
	} else {
		color = LIGHTGREY;
	}
	ILI9341_Draw_Text_Space("BOVT", 215, 160, color, Font_26_B, WHITE, 25);

}
#endif /* ENABLE_LCD */

void updateLCD() {
#ifdef ENABLE_LCD
	taskENTER_CRITICAL();
	drawBattery(jk_bms_battery_info.battery_status.battery_soc);
	drawVoltage_Current();

	ILI9341_Draw_Filled_Rectangle_Coord(310, 5, 315, 150, RED);

	if (lcd_temp != (jk_bms_battery_info.battery_status.sensor_temperature_1 + jk_bms_battery_info.battery_status.sensor_temperature_2)) {
		drawTemperature();
		lcd_temp = jk_bms_battery_info.battery_status.sensor_temperature_1 + jk_bms_battery_info.battery_status.sensor_temperature_2;
	}

	if (lcd_alarms != jk_bms_battery_info.battery_alarms.alarm_data) {
		lcd_alarms = jk_bms_battery_info.battery_alarms.alarm_data;
		drawAlarms();
	}
	taskEXIT_CRITICAL();
#endif
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startGetBMSDataTask */
/**
  * @brief  Function implementing the getBMSDataTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startGetBMSDataTask */
void startGetBMSDataTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  //uint8_t reset_count = 0;
  //updateLCD();
	/* Infinite loop */
  for(;;)
  {
	  osDelay(1000);
	  if (dataReady = 5) {
		  updateLCD();
		  dataReady = 0;
	  }
	  // Force new query every 5 seconds
	  if (UART_Rx_Current_Size == 0) {
		  UART_Rx_Size = 0;
		  HAL_UARTEx_ReceiveToIdle_IT(&huart1, UART_Rx_Buffer, 350);
		  Request_JK_Battery_485_Status_Frame(huart1, protocol_type);
		  //HAL_GPIO_TogglePin(LED_D2_GPIO_Port, LED_D2_Pin); //Toggle the state of pin
	  }
	  HAL_GPIO_TogglePin(LED_D2_GPIO_Port, LED_D2_Pin); //Toggle the state of pin
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startEvery10msTask */
/**
* @brief Function implementing the every10msTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startEvery10msTask */
void startEvery10msTask(void *argument)
{
  /* USER CODE BEGIN startEvery10msTask */
  /* Infinite loop */
  for(;;)
  {
	  if (UART_Rx_Current_Size > 0) {
		#ifdef __ENABLE_CONSOLE_DEBUG__
		  printf("------------------------------------\r\n");
		  for (int i=0; i<20; i++) {
			  for (int j=0; j<16; j++) {
				  printf("%02X ", UART_Rx_Buffer[16*i + j]);
			 }
			 printf("\r\n");
		  }
		#endif /* __ENABLE_CONSOLE_DEBUG__ */
		  Parse_JK_Battery_485_Status_Frame(&UART_Rx_Buffer, protocol_type);
		#ifdef __ENABLE_CONSOLE_DEBUG__
		  printf("Decoded data, protocol type: %u\r\n", protocol_type);
		  printf("Cells number: %u\r\n", jk_bms_battery_info.cells_number);
		  printf("Power tube temperature: %i\r\n", jk_bms_battery_info.battery_status.power_tube_temperature);
		  printf("Battery temperature 1: %i\r\n", jk_bms_battery_info.battery_status.sensor_temperature_1);
		  printf("Battery temperature 2: %i\r\n", jk_bms_battery_info.battery_status.sensor_temperature_2);
		  printf("Battery voltage: %u\r\n", jk_bms_battery_info.battery_status.battery_voltage);
		  printf("Battery current: %i\r\n", jk_bms_battery_info.battery_status.battery_current);

		  printf("Battery current bytes: Hi: %u Lo: %u\r\n", jk_bms_battery_info.battery_status.current_hi_byte, jk_bms_battery_info.battery_status.current_low_byte);

		  printf("SOC: %u\r\n", jk_bms_battery_info.battery_status.battery_soc);
		  printf("Battery cycles: %i\r\n", jk_bms_battery_info.battery_status.battery_cycles);
		  printf("Battery cycle capacity: %lu\r\n", jk_bms_battery_info.battery_status.battery_cycle_capacity);
		  printf("Battery alarms 0x%X\r\n", jk_bms_battery_info.battery_alarms.alarm_data);
		  printf("Battery charge voltage: %u\r\n", jk_bms_battery_info.battery_limits.battery_charge_voltage);
		  printf("Battery discharge voltage: %u\r\n", jk_bms_battery_info.battery_limits.battery_discharge_voltage);
		  printf("Battery charge current limit: %i\r\n", jk_bms_battery_info.battery_limits.battery_charge_current_limit);
		  printf("Battery discharge current limit: %i\r\n", jk_bms_battery_info.battery_limits.battery_discharge_current_limit);
		#endif /* __ENABLE_CONSOLE_DEBUG__ */
		  for (int i=0; i<200; i++) {
		   UART_Rx_Buffer[i] = 0;
		  }
		  Tx_JK_BMS_Status_via_CAN(hcan1);
		  UART_Rx_Current_Size = 0;
		  dataReady += 1;
	  }
	  osDelay(100);
  }
  /* USER CODE END startEvery10msTask */
}

/* USER CODE BEGIN Header_startConsoleOutputTask */

/**
* @brief Function implementing the consoleOutputTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startConsoleOutputTask */
void startConsoleOutputTask(void *argument)
{
  /* USER CODE BEGIN startConsoleOutputTask */
  /* Infinite loop */
	uint8_t count = 0;
	for(;;)
	{
		count = USB_Console_TX_Buffer_Count;
		osDelay(200);
		taskENTER_CRITICAL();
		if (count > 0 && USB_Console_TX_Buffer_Count == count) {
			CDC_Transmit_FS(USB_Console_TX_Buffer, USB_Console_TX_Buffer_Count);
			USB_Console_TX_Buffer_Count = 0;
		}
		taskEXIT_CRITICAL();
	}
  /* USER CODE END startConsoleOutputTask */
}

/* USER CODE BEGIN Header_startProcess_485_Request_Task */
/**
* @brief Function implementing the process_485_Request_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startProcess_485_Request_Task */
void startProcess_485_Request_Task(void *argument)
{
  /* USER CODE BEGIN startProcess_485_Request_Task */
  /* Infinite loop */
  for(;;)
  {
	  if (rs485_Pylon_data_ready > 0) {
		  #ifdef __ENABLE_CONSOLE_DEBUG__
		  	  printf("----------- PYLON RS485 Request ---------------------\r\n");
			  for (int i=0; i<rs485_Pylon_data_size; i++) {
				  printf("%02X ", UART_Rx_RS485_PYLON_Buffer[i]);
			  }

			  struct pylon_rs485_frame frame = Pylon_485_decode_frame(UART_Rx_RS485_PYLON_Buffer, rs485_Pylon_data_size);
			  printf("CRC %04X\r\n", frame.crc);
			  printf("Frame command: %02X\r\n", frame.cid2);
			  pylon_rs485_process_request(huart2, &frame);
 	 	  #endif /* __ENABLE_CONSOLE_DEBUG__ */
		  printf("\r\n");
		  rs485_Pylon_data_ready = 0;
		  rs485_Pylon_data_size = 0;
		  HAL_UARTEx_ReceiveToIdle_IT(&huart2, UART_Rx_RS485_PYLON_Buffer, 32);
	  }
    osDelay(10);
  }
  /* USER CODE END startProcess_485_Request_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

