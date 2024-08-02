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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "network.h"
#include "network_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

#define INTERVAL_IN_SECONDS 10 	//measurements interval
#define FLOOR_AREA 10 			//floor area of measured room
#define ROOM_NUMBER 0
#define OCCUPANCY 0
#define DISPLAY_LENGTH 190
#define WINDOW_SIZE 4

/* uncomment if you want to generate your own fake data as input */
//#define FAKE_DATA
/* uncomment if you are using time features in your model */
//#define TIME_FEATURES
/* uncomment if you are using acceleration features in your model */
//#define ACCELERATION_FEATURES

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
SensorData data = {0};

const char SCD30_INIT[] = "\r\n***  SCD30 Initialized ***\r\n";
const char READY_TIMEOUT[] = "\r\nTimeout waiting for data_ready flag\r\n";
const char MEASURE_ERROR[] = "\r\nerror reading measurement\r\n";
const char OFFICE[] = "Please type the office number: ";
const char OCCUPANTS[] = "Please type the number of occupants\r\n";
const char UART_ERROR[] = "UART failure\r\n";
const char UART_OK[] = "\r\nUART_OKAY\r\n";
//const char COLUMNS[] = "\r\ndatetime,temperature,humidity,co2_concentration,average_co2_last_hour,average_temperature_last_hour,average_humidity_last_hour,current_value_minus_average_last_hour_co2,current_value_minus_average_last_hour_temperature,current_value_minus_average_last_hour_humidity,current_value_minus_last_15_min_co2,current_value_minus_last_15_min_temperature,current_value_minus_last_15_min_humidity,acceleration_co2,acceleration_temperature,acceleration_humidity,occupancy";
const char COLUMNS[] = "\r\ndatetime	  	   CO2		    Temperature  		Humidity";
const char AI_INIT[] = "\r\n***** AI Initialized *****\r\n";

uint8_t RxBuffer[4];
/* Global handle to reference the instantiated C-model */
static ai_handle network = AI_HANDLE_NULL;



/* Global c-array to handle the activations buffer */
AI_ALIGNED(32)
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

/* Array to store the data of the input tensor */
AI_ALIGNED(32)
static ai_float in_data[AI_NETWORK_IN_1_SIZE];
/* or static ai_u8 in_data[AI_NETWORK_IN_1_SIZE_BYTES]; */

/* c-array to store the data of the output tensor */
AI_ALIGNED(32)
static ai_float out_data[AI_NETWORK_OUT_1_SIZE];
/* static ai_u8 out_data[AI_NETWORK_OUT_1_SIZE_BYTES]; */

/* Array of pointer to manage the model's input/output tensors */
static ai_buffer *ai_input;
static ai_buffer *ai_output;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

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
	char buf[50];
	int buf_len = 0;
  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  /*timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
	  Error_Handler();
  }*/
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  // Enable global interrupts
	__enable_irq();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
	HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
	HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
	timeout = 0xFFFF;
	while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
	if ( timeout < 0 )
	{
		Error_Handler();
	}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

	// Initialize the Neural Network
	AI_Init();

	initialize_sensor_data(&data);

	char occupancy[5];
	float prediction;
	float decision_boundary = 0.5;// DECISION_BOUNDARY;
	int16_t err;
	uint8_t first_line = 1;


	Sensor_Init(&hi2c1, INTERVAL_IN_SECONDS, 20000u);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		uint16_t data_ready = is_data_ready(&hi2c1, &huart3, INTERVAL_IN_SECONDS);
		if (!data_ready) {
    	  /* Timeout waiting for data_ready flag */
			printf(READY_TIMEOUT);
			continue;
		}
		/* read measurements */
		err = populateSensorData(&data, &hi2c1, &hrtc);

		if (err != NO_ERROR) {
		  /* error reading measurement */
			printf(MEASURE_ERROR);
		} else {
			if (first_line) {
				// Print columns (measured features)
				printf(COLUMNS);
				// Set the flag to indicate it's not the first line
				//first_line = 0;
			}
			print_values(&data);
		}

#ifdef FAKE_DATA
		AI_FAKE_fill_input_buffer(&data, in_data);

#else
		AI_fill_input_buffer(&data, in_data);
#endif
		// Run inference
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		AI_Run(in_data, out_data);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // To measure inference time

		// Read prediction of neural network
		prediction = ((float *)out_data)[0];
		if (prediction > decision_boundary) {
			sprintf(occupancy, "yes");
		} else {
			sprintf(occupancy, "no ");
		}
		// Print output of neural network
		buf_len = sprintf(buf, "\r\nOccupancy? %s \r\n\r\nCertitude: %.1f%%\r\n\r\n\r\n",occupancy, prediction*100);
		HAL_UART_Transmit(&huart3, (uint8_t *)buf, buf_len, 100);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x103032C3;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x12;
  sTime.Minutes = 0x00;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  sDate.Month = RTC_MONTH_AUGUST;
  sDate.Date = 0x02;
  sDate.Year = 0x24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  HAL_NVIC_SetPriority(USART3_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  /* Enable the UART RX FIFO full interrupt */
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXFF);
  HAL_UART_Receive_IT(&huart3, (uint8_t*)&RxBuffer, 1);

  //HAL_UART_Receive_IT(&huart3, &data.occupancy, 1);
  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Retargets the C library printf function to the USART.
  *   None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
	if(HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF) != HAL_OK)
		Error_Handler();
	return ch;
}
/**
  * @brief Initializes sensor data structure
  * @param data Pointer to SensorData structure
  * @retval None
  */
void initialize_sensor_data(SensorData *data)
{
    // Initialize datetime
    data->datetime = "";

    // Initialize sensor values
    data->co2_concentration = 0.0;
    data->temperature = 0.0;
    data->humidity = 0.0;

    // Initialize average values
    data->average_co2_last_hour = 0.0;
    data->average_temperature_last_hour = 0.0;
    data->average_humidity_last_hour = 0.0;

    // Initialize difference from average values
    data->current_value_minus_average_last_hour_co2 = 0.0;
    data->current_value_minus_average_last_hour_temperature = 0.0;
    data->current_value_minus_average_last_hour_humidity = 0.0;

    // Initialize difference from last 15 minutes
    data->current_value_minus_last_15_min_co2 = 0.0;
    data->current_value_minus_last_15_min_temperature = 0.0;
    data->current_value_minus_last_15_min_humidity = 0.0;

    // Initialize acceleration
    data->acceleration_co2 = 0.0;
    data->acceleration_temperature = 0.0;
    data->acceleration_humidity = 0.0;

    // Initialize occupancy and buffer count
    data->occupancy = 0;
    data->buffer_count = 0;
}

/**
  * @brief Update rolling averages
  * @param data Pointer to SensorData structure
  * @param new_co2 New CO2 concentration value
  * @param new_temperature New temperature value
  * @param new_humidity New humidity value
  * @retval None
  */
void update_rolling_averages(SensorData *data, float new_co2, float new_temperature, float new_humidity)
{
    static float co2_window[WINDOW_SIZE] = {0.0};
    static float temp_window[WINDOW_SIZE] = {0.0};
    static float hum_window[WINDOW_SIZE] = {0.0};
    static int index = 0;

    // Update averages only if there are enough elements in the buffer
    if (data->buffer_count == WINDOW_SIZE) {
        // Update averages
        float sum_co2 = 0.0, sum_temp = 0.0, sum_hum = 0.0;
        for (int i = 0; i < WINDOW_SIZE; ++i) {
            sum_co2 += co2_window[i];
            sum_temp += temp_window[i];
            sum_hum += hum_window[i];
        }

        data->average_co2_last_hour = sum_co2 / WINDOW_SIZE;
        data->average_temperature_last_hour = sum_temp / WINDOW_SIZE;
        data->average_humidity_last_hour = sum_hum / WINDOW_SIZE;

        // Update other features
        data->current_value_minus_average_last_hour_co2 = data->co2_concentration - data->average_co2_last_hour;
        data->current_value_minus_average_last_hour_temperature = data->temperature - data->average_temperature_last_hour;
        data->current_value_minus_average_last_hour_humidity = data->humidity - data->average_humidity_last_hour;
    } else {
        // If there aren't enough elements, initialize averages to current values
        data->average_co2_last_hour = new_co2;
        data->average_temperature_last_hour = new_temperature;
        data->average_humidity_last_hour = new_humidity;

        // Set other features to zero or default values
        data->current_value_minus_average_last_hour_co2 = 0.0;
        data->current_value_minus_average_last_hour_temperature = 0.0;
        data->current_value_minus_average_last_hour_humidity = 0.0;
    }
    // Increment buffer count if it's less than WINDOW_SIZE
    if (data->buffer_count < WINDOW_SIZE) {
        data->buffer_count++;
    }
    // Update the circular buffers
    co2_window[index] = new_co2;
    temp_window[index] = new_temperature;
    hum_window[index] = new_humidity;

    // Move to the next index (circular buffer)
    index = (index + 1) % WINDOW_SIZE;
}

/**
  * @brief Updates CO2, Temperature and Humidity accelerations in the SensorData structure
  * @param data Pointer to SensorData structure
  * @retval None
  */
void update_accelerations(SensorData *data, float old_co2_variation, float old_temperature_variation, float old_humidity_variation)
{
	// Update Accelerations
    data->acceleration_co2 =  data->current_value_minus_last_15_min_co2 - old_co2_variation;
    data->acceleration_temperature = data->current_value_minus_last_15_min_temperature - old_co2_variation;
    data->acceleration_humidity = data->current_value_minus_last_15_min_humidity - old_humidity_variation;
}

/**
  * @brief Update current values minus last measurement
  * @param data Pointer to SensorData structure
  * @param new_co2 New CO2 concentration value
  * @param new_temperature New temperature value
  * @param new_humidity New humidity value
  * @retval None
  */
void update_current_minus_last_measure(SensorData *data, float new_co2, float new_temperature, float new_humidity)
{
	float old_co2_variation = data->current_value_minus_last_15_min_co2;
	float old_temperature_variation = data->current_value_minus_last_15_min_temperature;
	float old_humidity_variation = data->current_value_minus_last_15_min_humidity;

	if (data->buffer_count > 1) {
		data->current_value_minus_last_15_min_co2 = new_co2 - data->co2_concentration;
	    data->current_value_minus_last_15_min_temperature = new_temperature - data->temperature;
	    data->current_value_minus_last_15_min_humidity = new_humidity - data->humidity;
	} else {
	    data->current_value_minus_last_15_min_co2 = 0.0;
	    data->current_value_minus_last_15_min_temperature = 0.0;
	    data->current_value_minus_last_15_min_humidity = 0.0;
	}

	update_accelerations(data, old_co2_variation, old_temperature_variation, old_humidity_variation);
}

/**
  * @brief Updates time-related features in the SensorData structure
  * @param data Pointer to SensorData structure
  * @param hrtc Pointer to RTC_HandleTypeDef structure
  * @retval None
  */
void update_time_features(SensorData *data, RTC_HandleTypeDef *hrtc)
{
    uint8_t datetime_length = 20;
    char datetime[datetime_length];

    RTC_TimeTypeDef time_struct;
    RTC_DateTypeDef date_struct;
    /* Get the RTC current Time and Date */
    HAL_RTC_GetTime(hrtc, &time_struct, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc, &date_struct, RTC_FORMAT_BIN);

    // Format datetime as dd-mm-yy hh:mm
    snprintf(datetime, datetime_length, "%02d-%02d-%02d %02d:%02d",
    		date_struct.Date, date_struct.Month, date_struct.Year, time_struct.Hours, time_struct.Minutes);

    // Allocate memory for datetime string
    data->datetime = (char*)malloc(datetime_length * sizeof(char));
    if (data->datetime != NULL) {
    	strcpy(data->datetime, datetime);
    }

    // Update other time-related features
    data->datetime_length = datetime_length;
    data->hour_of_the_day = time_struct.Hours;
    data->day_number_of_the_week = getDayOfWeek(date_struct);
    data->day_of_year = getDayOfYear(date_struct);
}

/**
  * @brief Populates sensor data based on measurements and time information
  * @param data Pointer to SensorData structure
  * @param hi2c Pointer to I2C_HandleTypeDef structure
  * @param hrtc Pointer to RTC_HandleTypeDef structure
  * @retval Error code (uint8_t)
  */
uint8_t populateSensorData(SensorData *data,  I2C_HandleTypeDef* hi2c, RTC_HandleTypeDef *hrtc)
{
	float new_co2 = 0.0, new_temperature = 0.0, new_humidity = 0.0;

	// Measure CO2, Temperature and Humidity
	uint8_t err = scd30_read_measurement(&hi2c1, &new_co2, &new_temperature, &new_humidity);

	// Update averages, variations and accelerations
	update_rolling_averages(data, new_co2, new_temperature, new_humidity);
	update_current_minus_last_measure(data, new_co2, new_temperature, new_humidity);

	// Update current values
	data->co2_concentration = new_co2;
	data->temperature = new_temperature;
	data->humidity = new_humidity;

	// Update time features
	update_time_features(data, hrtc);

	// Update other features
	data->floor_area = FLOOR_AREA;
    data->room_number = ROOM_NUMBER;

    // Update target value
    data->occupancy = OCCUPANCY;

    return err;
}

/**
  * @brief Prints sensor data to UART
  * @param huart Pointer to UART_HandleTypeDef structure
  * @param data Pointer to SensorData structure
  * @retval None
  */
void print_values(SensorData* data)
{
	char display_str[DISPLAY_LENGTH] = ""; // formated string for measurements

    // Create a formatted string with sensor data
	/*snprintf(display_str, DISPLAY_LENGTH, "\r\n%s , %.1f , %.1f , %.1f , %.2f , %.2f , %.2f , %.2f , %.2f , %.2f , %.2f , %.2f , %.2f , %.2f , %.2f , %.2f , %d \r\n",
			data->datetime,data->co2_concentration, data->temperature,  data->humidity,
			data->average_co2_last_hour, data->average_temperature_last_hour, data->average_humidity_last_hour,
			data->current_value_minus_average_last_hour_co2, data->current_value_minus_average_last_hour_temperature, data->current_value_minus_average_last_hour_humidity,
			data->current_value_minus_last_15_min_co2, data->current_value_minus_last_15_min_temperature, data->current_value_minus_last_15_min_temperature, data->acceleration_co2,
			data->acceleration_temperature, data->acceleration_humidity,
		    data->occupancy);
*/
	snprintf(display_str, DISPLAY_LENGTH, "\r\n%s 		%.1fppm 		%.1fdeg 		%.1f %% \r\n",
				data->datetime,data->co2_concentration, data->temperature,  data->humidity);
	printf(display_str);
}

/**
  * @brief Initializes the sensor and configures the SCD30 sensor.
  * @param hi2c Pointer to I2C_HandleTypeDef structure
  * @param measurement_interval Measurement interval in seconds
  * @param sleep_usec Sleep time in microseconds
  * @retval None
  */
void Sensor_Init(I2C_HandleTypeDef* hi2c, const int measurement_interval, const int sleep_usec)
{
    // Check I2C probes
    scd30_check_probe(hi2c);

    // Set measurement interval
    scd30_set_measurement_interval(hi2c, measurement_interval);

    // Sleep for the specified time
    sensirion_sleep_usec(sleep_usec);

    // Start periodic measurement
    scd30_start_periodic_measurement(hi2c, 0);

    printf(SCD30_INIT);
}


/**
  * @brief Initializes a neural network model and retrieves input/output pointers
  * @retval ai_error structure indicating success or specific error
  */
ai_error AI_Init(void)
{
	ai_error err = {.type = AI_ERROR_NONE,
					.code = AI_ERROR_CODE_NONE};

	/* Create and initialize the c-model */
	const ai_handle acts[] = { activations };
	err = ai_network_create_and_init(&network, acts, NULL);

	/* Retrieve pointers to the model's input/output tensors */
	ai_input = ai_network_inputs_get(network, NULL);
	ai_output = ai_network_outputs_get(network, NULL);

	printf(AI_INIT);

	return err;
}

/**
  * @brief Performs inference using the neural network model
  * @param in_data Pointer to input data (void pointer)
  * @param out_data Pointer to output data (void pointer)
  * @retval ai_error structure indicating success or specific error
  */
ai_error AI_Run(const void *in_data, void *out_data)
{
	ai_i32 n_batch;
	ai_error err = {.type = AI_ERROR_NONE,
					.code = AI_ERROR_CODE_NONE};

	/* Update IO handlers with the data payload */
	ai_input[0].data = AI_HANDLE_PTR(in_data);
	ai_output[0].data = AI_HANDLE_PTR(out_data);

	/* Perform the inference */
	n_batch = ai_network_run(network, &ai_input[0], &ai_output[0]);
	if (n_batch != 1) {
		err = ai_network_get_error(network);
	}

	return err;
}

/**
  * @brief Fills the input buffer with scaled sensor data
  * @param data Pointer to SensorData structure
  * @param in_data Pointer to the input buffer (ai_float*)
  * @retval None
  */
void AI_fill_input_buffer(SensorData *data, ai_float* in_data)
{
    uint8_t i = 0;

    // Scale and populate input data
    in_data[i++] = scale(data->temperature, MIN_TEMPERATURE, MAX_TEMPERATURE);
    in_data[i++] = scale(data->humidity, MIN_HUMIDITY, MAX_HUMIDITY);
    in_data[i++] = scale(data->co2_concentration, MIN_CO2_CONCENTRATION, MAX_CO2_CONCENTRATION);
    in_data[i++] = scale(data->average_co2_last_hour, MIN_AVERAGE_CO2_LAST_HOUR, MAX_AVERAGE_CO2_LAST_HOUR);
    in_data[i++] = scale(data->average_temperature_last_hour, MIN_AVERAGE_TEMPERATURE_LAST_HOUR, MAX_AVERAGE_TEMPERATURE_LAST_HOUR);
    in_data[i++] = scale(data->average_humidity_last_hour, MIN_AVERAGE_HUMIDITY_LAST_HOUR, MAX_AVERAGE_HUMIDITY_LAST_HOUR);
    in_data[i++] = scale(data->current_value_minus_average_last_hour_co2, MIN_CURRENT_VALUE_MINUS_AVERAGE_LAST_HOUR_CO2, MAX_CURRENT_VALUE_MINUS_AVERAGE_LAST_HOUR_CO2);
    in_data[i++] = scale(data->current_value_minus_average_last_hour_temperature, MIN_CURRENT_VALUE_MINUS_AVERAGE_LAST_HOUR_TEMPERATURE, MAX_CURRENT_VALUE_MINUS_AVERAGE_LAST_HOUR_TEMPERATURE);
    in_data[i++] = scale(data->current_value_minus_average_last_hour_humidity, MIN_CURRENT_VALUE_MINUS_AVERAGE_LAST_HOUR_HUMIDITY, MAX_CURRENT_VALUE_MINUS_AVERAGE_LAST_HOUR_HUMIDITY);
    in_data[i++] = scale(data->current_value_minus_last_15_min_co2, MIN_CURRENT_VALUE_MINUS_LAST_15_MIN_CO2, MAX_CURRENT_VALUE_MINUS_LAST_15_MIN_CO2);
    in_data[i++] = scale(data->current_value_minus_last_15_min_temperature, MIN_CURRENT_VALUE_MINUS_LAST_15_MIN_TEMPERATURE, MAX_CURRENT_VALUE_MINUS_LAST_15_MIN_TEMPERATURE);
    in_data[i++] = scale(data->current_value_minus_last_15_min_humidity, MIN_CURRENT_VALUE_MINUS_LAST_15_MIN_HUMIDITY, MAX_CURRENT_VALUE_MINUS_LAST_15_MIN_HUMIDITY);
#ifdef ACCELERATION_FEATURES
    in_data[i++] = scale(data->acceleration_co2, MIN_ACCELERATION_CO2, MAX_ACCELERATION_CO2);
    in_data[i++] = scale(data->acceleration_temperature, MIN_ACCELERATION_TEMPERATURE, MAX_ACCELERATION_TEMPERATURE);
    in_data[i++] = scale(data->acceleration_humidity, MIN_ACCELERATION_HUMIDITY, MAX_ACCELERATION_HUMIDITY);
#endif
    in_data[i++] = scale(data->room_number, MIN_ROOM_NUMBER, MAX_ROOM_NUMBER);

#ifdef TIME_FEATURES
    // Include time-related features if enabled
	in_data[i++] = scale(data->hour_of_the_day, MIN_HOUR_OF_THE_DAY, MAX_HOUR_OF_THE_DAY);
	in_data[i++] = scale(data->day_number_of_the_week, MIN_DAY_NUMBER_OF_THE_WEEK, MAX_DAY_NUMBER_OF_THE_WEEK);
	in_data[i++] = scale(data->day_of_year, MIN_DAY_OF_YEAR, MAX_DAY_OF_YEAR);
#endif
	// Scale floor area
	in_data[i++] = scale(data->floor_area, MIN_FLOOR_AREA, MAX_FLOOR_AREA);

}

/**
  * @brief Fills the input buffer with scaled FAKE data used for testing
  * @param data Pointer to SensorData structure
  * @param in_data Pointer to the input buffer (ai_float*)
  * @retval None
  */
void AI_FAKE_fill_input_buffer(SensorData *data, ai_float* in_data)
{
	//
	uint8_t i = 0;
	in_data[i++] = scale(26.1, MIN_TEMPERATURE, MAX_TEMPERATURE);
	in_data[i++] = scale(57.0 , MIN_HUMIDITY, MAX_HUMIDITY);
	in_data[i++] = scale(838.0, MIN_CO2_CONCENTRATION, MAX_CO2_CONCENTRATION);
	in_data[i++] = scale(874.38, MIN_AVERAGE_CO2_LAST_HOUR, MAX_AVERAGE_CO2_LAST_HOUR);
	in_data[i++] = scale(26.21, MIN_AVERAGE_TEMPERATURE_LAST_HOUR, MAX_AVERAGE_TEMPERATURE_LAST_HOUR);
	in_data[i++] = scale(57.82, MIN_AVERAGE_HUMIDITY_LAST_HOUR, MAX_AVERAGE_HUMIDITY_LAST_HOUR);
	in_data[i++] = scale(-24.88, MIN_CURRENT_VALUE_MINUS_AVERAGE_LAST_HOUR_CO2, MAX_CURRENT_VALUE_MINUS_AVERAGE_LAST_HOUR_CO2);
	in_data[i++] = scale(-0.10, MIN_CURRENT_VALUE_MINUS_AVERAGE_LAST_HOUR_TEMPERATURE, MAX_CURRENT_VALUE_MINUS_AVERAGE_LAST_HOUR_TEMPERATURE);
	in_data[i++] = scale(-0.40, MIN_CURRENT_VALUE_MINUS_AVERAGE_LAST_HOUR_HUMIDITY, MAX_CURRENT_VALUE_MINUS_AVERAGE_LAST_HOUR_HUMIDITY);
	in_data[i++] = scale(-11.48 , MIN_CURRENT_VALUE_MINUS_LAST_15_MIN_CO2, MAX_CURRENT_VALUE_MINUS_LAST_15_MIN_CO2);
	in_data[i++] = scale(-0.02, MIN_CURRENT_VALUE_MINUS_LAST_15_MIN_TEMPERATURE, MAX_CURRENT_VALUE_MINUS_LAST_15_MIN_TEMPERATURE);
	in_data[i++] = scale(-0.02, MIN_CURRENT_VALUE_MINUS_LAST_15_MIN_HUMIDITY, MAX_CURRENT_VALUE_MINUS_LAST_15_MIN_HUMIDITY);
#ifdef ACCELERATION_FEATURES
    in_data[i++] = scale(0, MIN_ACCELERATION_CO2, MAX_ACCELERATION_CO2);
    in_data[i++] = scale(0, MIN_ACCELERATION_TEMPERATURE, MAX_ACCELERATION_TEMPERATURE);
    in_data[i++] = scale(0, MIN_ACCELERATION_HUMIDITY, MAX_ACCELERATION_HUMIDITY);
#endif
	in_data[i++] = scale(0 , MIN_ROOM_NUMBER, MAX_ROOM_NUMBER);
#ifdef TIME_FEATURES
	in_data[i++] = scale(18, MIN_HOUR_OF_THE_DAY, MAX_HOUR_OF_THE_DAY);
	in_data[i++] = scale(3, MIN_DAY_NUMBER_OF_THE_WEEK, MAX_DAY_NUMBER_OF_THE_WEEK);
	in_data[i++] = scale(204, MIN_DAY_OF_YEAR, MAX_DAY_OF_YEAR);
#endif
	in_data[i++] = scale(10, MIN_FLOOR_AREA, MAX_FLOOR_AREA);
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
  //__disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_2);
	  HAL_Delay(1000);
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
