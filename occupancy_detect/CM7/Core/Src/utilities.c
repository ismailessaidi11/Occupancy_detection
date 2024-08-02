/*
 * utilities.c
 *
 *  Created on: Jul 9, 2024
 *      Author: ismailessaidi11
 */
#include "utilities.h"

// Function to check if a year is a leap year
uint8_t isLeapYear(int year) {
    if (year % 4 == 0) {
        if (year % 100 == 0) {
            if (year % 400 == 0) {
                return 1;
            } else {
                return 0;
            }
        } else {
            return 1;
        }
    } else {
        return 0;
    }
}

// Function to get the day of the year
uint8_t getDayOfYear(RTC_DateTypeDef sdatestructureget) {
    int month_days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int day_of_year = 0;
    int i;

    if (isLeapYear(2000 + sdatestructureget.Year)) {
        month_days[1] = 29; // February has 29 days in a leap year
    }

    for (i = 0; i < sdatestructureget.Month - 1; i++) {
        day_of_year += month_days[i];
    }

    day_of_year += sdatestructureget.Date;

    return day_of_year;
}

// Function to get the day of the week (1 = Sunday, 2 = Monday, ..., 7 = Saturday)
// Using Zeller's Congruence algorithm
uint8_t getDayOfWeek(RTC_DateTypeDef sdatestructureget) {
    int d = sdatestructureget.Date;
    int m = sdatestructureget.Month;
    int y = 2000 + sdatestructureget.Year;

    if (m < 3) {
        m += 12;
        y -= 1;
    }

    int K = y % 100;
    int J = y / 100;

    int f = d + 13*(m + 1)/5 + K + K/4 + J/4 + 5*J;
    int day_of_week = f % 7 + 1;

    return day_of_week;
}

// Function to perform min-max scaling and returns a float between 0 and 1
float scale(float value, float min, float max)
{
	if(min == max) {
		return 0.0;
	}
    return (value - min) / (max - min);
}
