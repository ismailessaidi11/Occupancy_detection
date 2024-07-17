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

#ifndef SENSIRION_COMMON_H
#define SENSIRION_COMMON_H

#include "sensirion_arch_config.h"
#include "stm32h7xx_hal.h"
#include "scd30_utils.h"

#ifdef __cplusplus
extern "C" {
#endif


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

// Private macro
#define countof(a) (sizeof(a) / sizeof(*(a)))


uint16_t scd30_start_periodic_measurement(I2C_HandleTypeDef* hi2c, uint16_t ambient_pressure_mbar);
int16_t scd30_stop_periodic_measurement(I2C_HandleTypeDef* hi2c);
int16_t scd30_read_measurement(I2C_HandleTypeDef* hi2c, float* co2_ppm, float* temperature,
                               float* humidity);
int16_t scd30_set_measurement_interval(I2C_HandleTypeDef* hi2c, uint16_t interval_sec);
int16_t scd30_get_data_ready(I2C_HandleTypeDef* hi2c, uint16_t* data_ready);
int16_t scd30_set_temperature_offset(I2C_HandleTypeDef* hi2c, uint16_t temperature_offset);
int16_t scd30_probe(I2C_HandleTypeDef* hi2c);

uint16_t is_data_ready(I2C_HandleTypeDef* hi2c,UART_HandleTypeDef* huart, uint16_t interval_in_seconds);
void error_Handler(void);
#ifdef __cplusplus
}
#endif

#endif /* SENSIRION_COMMON_H */
