/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "utilities.h"
#include <sensor_Inc/scd30.h>
#include "syscall.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* AI */
#include "ai_datatypes_defines.h"
#include "ai_platform.h"
#include "AI_constants.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    char* datetime;
    uint8_t datetime_length;
    float temperature;
    float co2_concentration;
    float humidity;
    float average_co2_last_hour;
    float average_temperature_last_hour;
    float average_humidity_last_hour;
    float current_value_minus_average_last_hour_co2;
    float current_value_minus_average_last_hour_temperature;
    float current_value_minus_average_last_hour_humidity;
    float current_value_minus_last_15_min_co2;
    float current_value_minus_last_15_min_temperature;
    float current_value_minus_last_15_min_humidity;
    float acceleration_co2;
	float acceleration_temperature;
    float acceleration_humidity;
    uint8_t room_number;
    uint8_t hour_of_the_day;
    uint8_t day_number_of_the_week;
    uint8_t day_of_year;
    uint8_t floor_area;
    uint8_t occupancy;
    uint8_t buffer_count; // Number of elements currently in the circular buffer (for average computation)
} SensorData;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void initialize_sensor_data(SensorData *data);
void update_rolling_averages(SensorData *data, float new_co2, float new_temperature, float new_humidity);
void update_current_minus_last_measure(SensorData *data, float new_co2, float new_temperature, float new_humidity);
void update_accelerations(SensorData *data, float old_co2_variation, float old_temperature_variation, float old_humidity_variation);
void update_time_features(SensorData *data, RTC_HandleTypeDef *hrtc);
uint8_t populateSensorData(SensorData *data, I2C_HandleTypeDef* hi2c, RTC_HandleTypeDef *hrtc);
void print_values(SensorData* data);
void copySensorData(SensorData *dest, const SensorData *src);
void Sensor_Init(I2C_HandleTypeDef* hi2c, const int measurement_interval, const int sleep_usec);
ai_error AI_Init(void);
ai_error AI_Run(const void *in_data, void *out_data);
void AI_fill_input_buffer(SensorData *data, ai_float* in_data);
void AI_FAKE_fill_input_buffer(SensorData *data, ai_float* in_data);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOJ

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
