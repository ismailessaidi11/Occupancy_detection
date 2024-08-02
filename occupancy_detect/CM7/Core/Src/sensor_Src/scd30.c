#include <sensor_Inc/scd30.h>
/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 *
 * This module provides functionality that is common to all Sensirion drivers
 */


const char PROBING_FAIL[] = "\r\nSCD30 sensor probing failed\r\n";
const char PROBING_SUCCESS[] = "\r\nSCD30 sensor probing successful\n\n\r";

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

int16_t scd30_set_temperature_offset(I2C_HandleTypeDef* hi2c, uint16_t temperature_offset)
{
    int16_t error;

    error = sensirion_i2c_write_cmd_with_args(hi2c,
        SCD30_I2C_ADDRESS, SCD30_CMD_SET_TEMPERATURE_OFFSET,
        &temperature_offset, SENSIRION_NUM_WORDS(temperature_offset));
    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);

    return error;
}

void scd30_check_probe(I2C_HandleTypeDef* hi2c)
{
    uint16_t data_ready;

    /* try to read data-ready state */
    while (scd30_get_data_ready(hi2c, &data_ready) != NO_ERROR)
    {
    	HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_2);
    	printf(PROBING_FAIL);
    	sensirion_sleep_usec(1000000u);
    }
	// probing was successful
	//printf(PROBING_SUCCESS);
}


uint16_t is_data_ready(I2C_HandleTypeDef* hi2c,UART_HandleTypeDef* huart, uint16_t interval_in_seconds)
{
	uint8_t NOT_READY[] = "\r\nError reading data_ready flag\r\n ";
	uint16_t data_ready = 0;
    uint16_t timeout = 0;
    int16_t err;
    /* Poll data_ready flag until data is available. Allow 20% more than
     * the measurement interval to account for clock imprecision of the
     * sensor.
     */
    for (timeout = 0; (100000 * timeout) < (interval_in_seconds * 1200000); ++timeout) {
        err = scd30_get_data_ready(hi2c, &data_ready);
        if (err != NO_ERROR) {
      	  HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_2);
      	  /* Error reading data_ready flag */
      	  if (HAL_UART_Transmit(huart, NOT_READY, countof(NOT_READY)-1, 1000) != HAL_OK)
      		  error_Handler();
        }
        if (data_ready) {
            return data_ready;
        }
        sensirion_sleep_usec(100000);
    }
	return data_ready;
}

void error_Handler(void)
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
