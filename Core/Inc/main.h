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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
uint16_t scd30_start_periodic_measurement(I2C_HandleTypeDef* hi2c, uint16_t ambient_pressure_mbar);
int16_t scd30_stop_periodic_measurement(I2C_HandleTypeDef* hi2c);
int16_t scd30_read_measurement(I2C_HandleTypeDef* hi2c, float* co2_ppm, float* temperature,
                               float* humidity);
int16_t scd30_set_measurement_interval(I2C_HandleTypeDef* hi2c, uint16_t interval_sec);
int16_t scd30_get_data_ready(I2C_HandleTypeDef* hi2c, uint16_t* data_ready);
int16_t scd30_set_temperature_offset(I2C_HandleTypeDef* hi2c, uint16_t temperature_offset);
int16_t scd30_set_altitude(I2C_HandleTypeDef* hi2c, uint16_t altitude);
int16_t scd30_get_automatic_self_calibration(I2C_HandleTypeDef* hi2c, uint8_t* asc_enabled);
int16_t scd30_enable_automatic_self_calibration(I2C_HandleTypeDef* hi2c, uint8_t enable_asc);
int16_t scd30_set_forced_recalibration(I2C_HandleTypeDef* hi2c, uint16_t co2_ppm);
int16_t scd30_read_serial(I2C_HandleTypeDef* hi2c, char* serial);
uint8_t scd30_get_configured_address();
int16_t scd30_probe(I2C_HandleTypeDef* hi2c);

void display_values(char* display_str, float temperature, float co2_ppm, float relative_humidity, uint8_t office_number, uint8_t occupancy);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
