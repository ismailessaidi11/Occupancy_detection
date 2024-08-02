/*
 * utilities.h
 *
 *  Created on: Jul 9, 2024
 *      Author: ismailessaidi11
 */

#ifndef INC_UTILITIES_H_
#define INC_UTILITIES_H_


#include "stm32h7xx_hal.h"
#include <stdio.h>


// Function to check if a year is a leap year
uint8_t isLeapYear(int year);

// Function to get the day of the year
uint8_t getDayOfYear(RTC_DateTypeDef sdatestructureget);

// Function to get the day of the week (0 = Sunday, 1 = Monday, ..., 6 = Saturday)
// Using Zeller's Congruence algorithm
uint8_t getDayOfWeek(RTC_DateTypeDef sdatestructureget);

// Function to perform min-max scaling and returns a float between 0 and 1
float scale(float value, float min, float max);


#endif /* INC_UTILITIES_H_ */
