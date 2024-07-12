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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

#define INTERVAL_IN_SECONDS 60 //measurements interval
#define FLOOR_AREA 10 			//floor area of measured room
#define ROOM_NUMBER 0
#define OCCUPANCY 1
#define DISPLAY_LENGTH 190
#define WINDOW_SIZE 4

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

uint8_t INIT[] = "***** Initialize SCD30 sensor *****\r\n";
uint8_t PROBING_FAIL[] = "SCD30 sensor probing failed\r\n";
uint8_t PROBING_SUCCESS[] = "SCD30 sensor probing successful\n\n\r";

uint8_t READY_TIMEOUT[] = "\r\nTimeout waiting for data_ready flag\r\n";
uint8_t MEASURE_ERROR[] = "\r\nerror reading measurement\r\n";
uint8_t OFFICE[] = "Please type the office number: ";
uint8_t OCCUPANTS[] = "Please type the number of occupants\r\n";
uint8_t UART_ERROR[] = "UART failure\r\n";
uint8_t UART_OK[] = "\r\nUART_OKAY\r\n";
uint8_t COLUMNS[] = "\r\ndatetime,temperature,humidity,co2_concentration,average_co2_last_hour,average_temperature_last_hour,average_humidity_last_hour,current_value_minus_average_last_hour_co2,current_value_minus_average_last_hour_temperature,current_value_minus_average_last_hour_humidity,current_value_minus_last_15_min_co2,current_value_minus_last_15_min_temperature,current_value_minus_last_15_min_humidity,room_number,hour_of_the_day,day_number_of_the_week,day_of_year,floor_area,occupancy";

uint8_t RxBuffer[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

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

  initialize_sensor_data(&data);

  int16_t err;
  uint8_t first_line = 1;
  //if (HAL_UART_Transmit(&huart3, INIT, countof(INIT)-1, 1000) != HAL_OK)
//	  Error_Handler();

  /* Busy loop for initialization, because the main loop does not work without
   * a sensor.
   */
  while (scd30_probe(&hi2c1) != NO_ERROR) {
	  HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_2);
	  // probing failed
	  if (HAL_UART_Transmit(&huart3, PROBING_FAIL, countof(PROBING_FAIL)-1, 1000) != HAL_OK)
		  Error_Handler();
      sensirion_sleep_usec(1000000u);
  }
  // probing was successful

  //if (HAL_UART_Transmit(&huart3, PROBING_SUCCESS, countof(PROBING_SUCCESS)-1, 1000) != HAL_OK)
//	  Error_Handler();


  scd30_set_measurement_interval(&hi2c1, INTERVAL_IN_SECONDS);
  sensirion_sleep_usec(20000u);
  scd30_start_periodic_measurement(&hi2c1, 0);
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
		  if (HAL_UART_Transmit(&huart3, READY_TIMEOUT, countof(READY_TIMEOUT)-1, 1000) != HAL_OK)
			  Error_Handler();
          continue;
      }
      /* read measurements */
      err = populateSensorData(&data, &hi2c1, &hrtc);

      if (err != NO_ERROR) {
          /* error reading measurement */
    	  if (HAL_UART_Transmit(&huart3, MEASURE_ERROR, countof(MEASURE_ERROR)-1, 1000) != HAL_OK)
			  Error_Handler();

      } else {
          /* print values */
    	  if (first_line)
    		  //print_columns(&huart3, &first_line);

    	  print_values(&huart3, &data);
      }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x403032CA;
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
  sTime.Minutes = 0x52;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  sDate.Month = RTC_MONTH_JULY;
  sDate.Date = 0x12;
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
  huart3.Init.Parity = UART_PARITY_ODD;
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
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

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

// Initialize sensor data
void initialize_sensor_data(SensorData *data)
{
	data->datetime = "";
    data->co2_concentration = 0.0;
    data->temperature = 0.0;
    data->humidity = 0.0;
    data->average_co2_last_hour = 0.0;
    data->average_temperature_last_hour = 0.0;
    data->average_humidity_last_hour = 0.0;
    data->current_value_minus_average_last_hour_co2 = 0.0;
    data->current_value_minus_average_last_hour_temperature = 0.0;
    data->current_value_minus_average_last_hour_humidity = 0.0;
    data->current_value_minus_last_15_min_co2 = 0.0;
    data->current_value_minus_last_15_min_temperature = 0.0;
    data->current_value_minus_last_15_min_humidity = 0.0;
    data->occupancy = 5;
    data->buffer_count = 0;
}
// Update rolling averages
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

// Update current values minus last measurement
void update_current_minus_last_measure(SensorData *data, float new_co2, float new_temperature, float new_humidity)
{
	if (data->buffer_count > 1) {
	    data->current_value_minus_last_15_min_co2 = new_co2 - data->co2_concentration;
	    data->current_value_minus_last_15_min_temperature = new_temperature - data->temperature;
	    data->current_value_minus_last_15_min_humidity = new_humidity - data->humidity;
	} else {
	    data->current_value_minus_last_15_min_co2 = 0.0;
	    data->current_value_minus_last_15_min_temperature = 0.0;
	    data->current_value_minus_last_15_min_humidity = 0.0;
	}
}

uint8_t populateSensorData(SensorData *data,  I2C_HandleTypeDef* hi2c, RTC_HandleTypeDef *hrtc)
{
	float new_co2 = 0.0, new_temperature = 0.0, new_humidity = 0.0;

	uint8_t err = scd30_read_measurement(&hi2c1, &new_co2, &new_temperature, &new_humidity);

	update_rolling_averages(data, new_co2, new_temperature, new_humidity);
	update_current_minus_last_measure(data, new_co2, new_temperature, new_humidity);

	// Update current values
	data->co2_concentration = new_co2;
	data->temperature = new_temperature;
	data->humidity = new_humidity;

	/*************************************** update time features **************************************/
    uint8_t datetime_length = 20;
    char datetime[datetime_length];

    RTC_TimeTypeDef time_struct;
    RTC_DateTypeDef date_struct;
    /* Get the RTC current Time */
    HAL_RTC_GetTime(hrtc, &time_struct, RTC_FORMAT_BIN);
    /* Get the RTC current Date */
    HAL_RTC_GetDate(hrtc, &date_struct, RTC_FORMAT_BIN);

    // formate datetime dd-mm-yy hh:mm
    snprintf(datetime, datetime_length, "%02d-%02d-%02d %02d:%02d",
    		date_struct.Date, date_struct.Month, date_struct.Year, time_struct.Hours, time_struct.Minutes);


    data->datetime = (char*)malloc(datetime_length * sizeof(char));
    if (data->datetime != NULL) {
    	strcpy(data->datetime, datetime);
    }

    data->datetime_length = datetime_length;
    data->hour_of_the_day = time_struct.Hours;
    data->day_number_of_the_week = getDayOfWeek(date_struct);
    data->day_of_year = getDayOfYear(date_struct);

    /*************************************** update other features **************************************/
    data->floor_area = FLOOR_AREA;
    data->room_number = ROOM_NUMBER;
    data->occupancy = OCCUPANCY;

    return err;
}

void print_values(UART_HandleTypeDef* huart, SensorData* data)
{
	char display_str[DISPLAY_LENGTH] = ""; // formated string for measurements

	/*snprintf(display_str, DISPLAY_LENGTH, "\r\n%s , ",
				data->datetime);
*/
	snprintf(display_str, DISPLAY_LENGTH, "\r\n%s , %.1f , %.1f , %.1f , %.2f , %.2f , %.2f , %.2f , %.2f , %.2f , %.2f , %.2f , %.2f , %d , %d , %d , %d , %d , %d\r",
			data->datetime, data->temperature,  data->humidity,data->co2_concentration,
			data->average_co2_last_hour, data->average_temperature_last_hour, data->average_humidity_last_hour,
			data->current_value_minus_average_last_hour_co2, data->current_value_minus_average_last_hour_temperature, data->current_value_minus_average_last_hour_humidity,
			data->current_value_minus_last_15_min_co2, data->current_value_minus_last_15_min_temperature, data->current_value_minus_last_15_min_temperature,
			data->room_number, data->hour_of_the_day, data->day_number_of_the_week,
			data->day_of_year, data->floor_area, data->occupancy);

	if (HAL_UART_Transmit(huart, (uint8_t *)display_str, countof(display_str)-1, 1000) != HAL_OK)
		Error_Handler();
}

void print_columns(UART_HandleTypeDef* huart, uint8_t* first_line)
{
	if (HAL_UART_Transmit(huart, (uint8_t *)COLUMNS, countof(COLUMNS)-1, 1000) != HAL_OK)
		Error_Handler();

	*first_line = 0;
}

// UART receive complete callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {

        // Start another reception
    	HAL_UART_Receive_IT(&huart3, (uint8_t*)&RxBuffer, 1);
        //HAL_UART_Receive_IT(&huart3, &data.occupancy, 1);
    }
}
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 if(recvd_data == '\r') //when enter is pressed go to this condition
	 {
		 data_buffer[count++]='\r';
		 HAL_UART_Transmit(huart,data_buffer,count,1000); //transmit the full sentence again
		 memset(data_buffer, 0, count); // empty the data buffer
		 count = 0;
	 }
	 else
	 {
		 data_buffer[count++] = recvd_data; // every time when interrupt is happen, received 1 byte of data
	 }
	 HAL_UART_Receive_IT(&huart3,&recvd_data,1); //start next data receive interrupt

}
*/
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
