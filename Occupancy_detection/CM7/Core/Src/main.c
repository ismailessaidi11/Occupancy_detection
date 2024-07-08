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
#include "scd30.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

#define DISPLAY_LENGTH 100

/* Slave I2C address */
#define SCD30_I2C_ADDRESS 0x61

/* Codes des commandes */
#define SCD30_CMD_START_PERIODIC_MEASUREMENT 0x0010
#define SCD30_CMD_STOP_PERIODIC_MEASUREMENT 0x0104
#define SCD30_CMD_READ_MEASUREMENT 0x0300
#define SCD30_CMD_SET_MEASUREMENT_INTERVAL 0x4600
#define SCD30_CMD_GET_DATA_READY 0x0202
#define SCD30_CMD_SET_TEMPERATURE_OFFSET 0x5403
#define SCD30_CMD_SET_ALTITUDE 0x5102
#define SCD30_CMD_SET_FORCED_RECALIBRATION 0x5204
#define SCD30_CMD_AUTO_SELF_CALIBRATION 0x5306
#define SCD30_CMD_READ_SERIAL 0xD033

#define SCD30_SERIAL_NUM_WORDS 16
#define SCD30_WRITE_DELAY_US 20000
#define SCD30_MAX_BUFFER_WORDS 24
#define SCD30_CMD_SINGLE_WORD_BUF_LEN \
    (SENSIRION_COMMAND_SIZE + SENSIRION_WORD_SIZE + CRC8_LEN)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define countof(a) (sizeof(a) / sizeof(*(a)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
char display_str[DISPLAY_LENGTH] = ""; // formated string for measurements
uint8_t INIT[] = "***** Initialize SCD30 sensor *****\r\n";
uint8_t PROBING_FAIL[] = "SCD30 sensor probing failed\r\n";
uint8_t PROBING_SUCCESS[] = "SCD30 sensor probing successful\n\n\r";
uint8_t NOT_READY[] = "\r\nError reading data_ready flag\r\n ";
uint8_t READY_TIMEOUT[] = "\r\nTimeout waiting for data_ready flag\r\n";
uint8_t MEASURE_ERROR[] = "\r\nerror reading measurement\r\n";
uint8_t OFFICE[] = "Please type the office number: ";
uint8_t OCCUPANTS[] = "Please type the number of occupants\r\n";
uint8_t UART_ERROR[] = "UART failure\r\n";
uint8_t UART_OK[] = "\r\nOKAY\r\n";

uint8_t office_number = 0;
uint8_t occupancy = 1;

uint8_t data_buffer[100]; // data buffer
uint8_t recvd_data; // receive buffer
uint32_t count=0; // count how many bytes are received

uint8_t aShowTime[50] = {0};
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

  float co2_ppm, temperature, relative_humidity;
  int16_t err;
  uint16_t interval_in_seconds = 2;//900;

  if (HAL_UART_Transmit(&huart3, INIT, countof(INIT)-1, 1000) != HAL_OK)
  {
	  Error_Handler();
  }

  /* Busy loop for initialization, because the main loop does not work without
   * a sensor.
   */
  while (scd30_probe(&hi2c1) != NO_ERROR) {
	  HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_2);
	  // probing failed
	  if (HAL_UART_Transmit(&huart3, PROBING_FAIL, countof(PROBING_FAIL)-1, 1000) != HAL_OK)
	  {
		  Error_Handler();
	  }
      sensirion_sleep_usec(1000000u);
  }
  // probing was successful

  if (HAL_UART_Transmit(&huart3, PROBING_SUCCESS, countof(PROBING_SUCCESS)-1, 1000) != HAL_OK)
  {
	  Error_Handler();
  }


  scd30_set_measurement_interval(&hi2c1, interval_in_seconds);
  sensirion_sleep_usec(20000u);
  scd30_start_periodic_measurement(&hi2c1, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint16_t data_ready = 0;
      uint16_t timeout = 0;

      /* Poll data_ready flag until data is available. Allow 20% more than
       * the measurement interval to account for clock imprecision of the
       * sensor.
       */
      for (timeout = 0; (100000 * timeout) < (interval_in_seconds * 1200000);
           ++timeout) {
          err = scd30_get_data_ready(&hi2c1, &data_ready);
          if (err != NO_ERROR) {
        	  HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_2);

        	  /* Error reading data_ready flag */
        	  if (HAL_UART_Transmit(&huart3, NOT_READY, countof(NOT_READY)-1, 1000) != HAL_OK)
        	  {
        		  Error_Handler();
        	  }
          }
          if (data_ready) {
              break;
          }
          sensirion_sleep_usec(100000);
      }
      if (!data_ready) {
    	  /* Timeout waiting for data_ready flag */
		  if (HAL_UART_Transmit(&huart3, READY_TIMEOUT, countof(READY_TIMEOUT)-1, 1000) != HAL_OK)
		  {
			  Error_Handler();
		  }
          continue;
      }

      /* Measure co2, temperature and relative humidity and store into
       * variables.
       */
      err =
          scd30_read_measurement(&hi2c1, &co2_ppm, &temperature, &relative_humidity);
      if (err != NO_ERROR) {
          /* error reading measurement */
    	  if (HAL_UART_Transmit(&huart3, MEASURE_ERROR, countof(MEASURE_ERROR)-1, 1000) != HAL_OK)
		  {
			  Error_Handler();
		  }

      } else {
          /* Display values */
    	  display_values(display_str, temperature, co2_ppm, relative_humidity, office_number, occupancy);
    	  if (HAL_UART_Transmit(&huart3, (uint8_t *)display_str, countof(display_str)-1, 1000) != HAL_OK)
    	  {
    		  Error_Handler();
    	  }
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
  sTime.Hours = 0x14;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 0x2;
  sDate.Year = 0x23;

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
  huart3.Init.BaudRate = 115200;
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
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

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

uint16_t scd30_start_periodic_measurement(I2C_HandleTypeDef* hi2c, uint16_t ambient_pressure_mbar) {
    if (ambient_pressure_mbar &&
        (ambient_pressure_mbar < 700 || ambient_pressure_mbar > 1400)) {
        /* out of allowable range */
        return STATUS_FAIL;
    }

    return sensirion_i2c_write_cmd_with_args(hi2c,
        SCD30_I2C_ADDRESS, SCD30_CMD_START_PERIODIC_MEASUREMENT,
        &ambient_pressure_mbar, SENSIRION_NUM_WORDS(ambient_pressure_mbar));
}

int16_t scd30_stop_periodic_measurement(I2C_HandleTypeDef* hi2c) {
    return sensirion_i2c_write_cmd(hi2c, SCD30_I2C_ADDRESS,
                                   SCD30_CMD_STOP_PERIODIC_MEASUREMENT);
}

int16_t scd30_read_measurement(I2C_HandleTypeDef* hi2c, float* co2_ppm, float* temperature,
                               float* humidity) {
    int16_t error;
    uint8_t data[3][4];

    error =
        sensirion_i2c_write_cmd(hi2c, SCD30_I2C_ADDRESS, SCD30_CMD_READ_MEASUREMENT);
    if (error != NO_ERROR)
        return error;

    error = sensirion_i2c_read_words_as_bytes(hi2c, SCD30_I2C_ADDRESS, &data[0][0],
                                              SENSIRION_NUM_WORDS(data));
    if (error != NO_ERROR)
        return error;

    *co2_ppm = sensirion_bytes_to_float(data[0]);
    *temperature = sensirion_bytes_to_float(data[1]);
    *humidity = sensirion_bytes_to_float(data[2]);

    return NO_ERROR;
}

int16_t scd30_set_measurement_interval(I2C_HandleTypeDef* hi2c, uint16_t interval_sec) {
    int16_t error;

    if (interval_sec < 2 || interval_sec > 1800) {
        /* out of allowable range */
        return STATUS_FAIL;
    }

    error = sensirion_i2c_write_cmd_with_args(hi2c,
        SCD30_I2C_ADDRESS, SCD30_CMD_SET_MEASUREMENT_INTERVAL, &interval_sec,
        SENSIRION_NUM_WORDS(interval_sec));
    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);

    return error;
}

int16_t scd30_get_data_ready(I2C_HandleTypeDef* hi2c, uint16_t* data_ready) {
    return sensirion_i2c_delayed_read_cmd(hi2c,
        SCD30_I2C_ADDRESS, SCD30_CMD_GET_DATA_READY, 3000, data_ready,
        SENSIRION_NUM_WORDS(*data_ready));
}

int16_t scd30_set_temperature_offset(I2C_HandleTypeDef* hi2c, uint16_t temperature_offset) {
    int16_t error;

    error = sensirion_i2c_write_cmd_with_args(hi2c,
        SCD30_I2C_ADDRESS, SCD30_CMD_SET_TEMPERATURE_OFFSET,
        &temperature_offset, SENSIRION_NUM_WORDS(temperature_offset));
    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);

    return error;
}

int16_t scd30_set_altitude(I2C_HandleTypeDef* hi2c, uint16_t altitude) {
    int16_t error;

    error = sensirion_i2c_write_cmd_with_args(hi2c, SCD30_I2C_ADDRESS,
                                              SCD30_CMD_SET_ALTITUDE, &altitude,
                                              SENSIRION_NUM_WORDS(altitude));
    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);

    return error;
}

int16_t scd30_get_automatic_self_calibration(I2C_HandleTypeDef* hi2c, uint8_t* asc_enabled) {
    uint16_t word;
    int16_t error;

    error = sensirion_i2c_read_cmd(hi2c, SCD30_I2C_ADDRESS,
                                   SCD30_CMD_AUTO_SELF_CALIBRATION, &word,
                                   SENSIRION_NUM_WORDS(word));
    if (error != NO_ERROR)
        return error;

    *asc_enabled = (uint8_t)word;

    return NO_ERROR;
}

int16_t scd30_enable_automatic_self_calibration(I2C_HandleTypeDef* hi2c, uint8_t enable_asc) {
    int16_t error;
    uint16_t asc = !!enable_asc;

    error = sensirion_i2c_write_cmd_with_args(hi2c, SCD30_I2C_ADDRESS,
                                              SCD30_CMD_AUTO_SELF_CALIBRATION,
                                              &asc, SENSIRION_NUM_WORDS(asc));
    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);

    return error;
}

int16_t scd30_set_forced_recalibration(I2C_HandleTypeDef* hi2c, uint16_t co2_ppm) {
    int16_t error;

    error = sensirion_i2c_write_cmd_with_args(hi2c,
        SCD30_I2C_ADDRESS, SCD30_CMD_SET_FORCED_RECALIBRATION, &co2_ppm,
        SENSIRION_NUM_WORDS(co2_ppm));
    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);

    return error;
}

int16_t scd30_read_serial(I2C_HandleTypeDef* hi2c, char* serial) {
    int16_t error;

    error = sensirion_i2c_write_cmd(hi2c, SCD30_I2C_ADDRESS, SCD30_CMD_READ_SERIAL);
    if (error)
        return error;

    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);
    error = sensirion_i2c_read_words_as_bytes(hi2c,
        SCD30_I2C_ADDRESS, (uint8_t*)serial, SCD30_SERIAL_NUM_WORDS);
    serial[2 * SCD30_SERIAL_NUM_WORDS] = '\0';
    return error;
}

uint8_t scd30_get_configured_address() {
    return SCD30_I2C_ADDRESS;
}

int16_t scd30_probe(I2C_HandleTypeDef* hi2c) {
    uint16_t data_ready;

    /* try to read data-ready state */
    return scd30_get_data_ready(hi2c, &data_ready);
}

void display_values(char* display_str, float temperature, float co2_ppm, float relative_humidity, uint8_t office_number, uint8_t occupancy)
{
	  int temperatureInt, temperatureFrac, humidityInt, humidityFrac;
	  float tmpFrac;

	  temperatureInt = temperature;
	  tmpFrac = temperature - temperatureInt;
	  temperatureFrac = trunc(tmpFrac * 10);

	  humidityInt = relative_humidity;
	  tmpFrac = relative_humidity - humidityInt;
	  humidityFrac = trunc(tmpFrac * 100);


	  RTC_DateTypeDef sdatestructureget;
	  RTC_TimeTypeDef stimestructureget;

	  /* Get the RTC current Time */
	  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	  /* Get the RTC current Date */
	  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

	  snprintf(display_str, DISPLAY_LENGTH,"\r\n%02d-%02d-%02d %02d:%02d \t %d.%01d \t %d \t %d.%02d \t %d   %d\r",
			  sdatestructureget.Date, sdatestructureget.Month, sdatestructureget.Year, stimestructureget.Hours, stimestructureget.Minutes,
			  temperatureInt, temperatureFrac, (int)co2_ppm, humidityInt, humidityFrac, (int)office_number, (int)occupancy);

}


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
